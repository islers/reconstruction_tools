/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of dense_reconstruction, a ROS package for...well,

dense_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
dense_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with dense_reconstruction. If not, see <http://www.gnu.org/licenses/>.
*/

#include <boost/program_options.hpp>
#include <boost/function.hpp>

#include <fstream>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "reconstruction_tools/model_completeness_calculator.h"
#include "dense_reconstruction/ViewInformationReturn.h"
#include "std_srvs/Empty.h"
#include "tf2_msgs/TFMessage.h"

using namespace std;

// UNDO TILL HERE
bool octomapPublished;
void octomapListening(const octomap_msgs::OctomapConstPtr&)
{
      octomapPublished = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "entropy_from_pcl_calculator");
  ros::NodeHandle n;
  
  boost::program_options::options_description desc("This node reads pcl data from a bag file, publishs it pointcloud by pointcloud, waiting for an octomap node (which must be started separately) to register it and then calls the entropy service of the octomap.\n\n");
  
  desc.add_options()
    ("help,h", "Prints this help.")
    ("bag,b", boost::program_options::value< std::vector<std::string > >(), "Path to the input bag from which pcl data is to be read. If you want to process more than one file, omit the qualifier and directly pass the list of bag files.")
    ("pclInTopic", boost::program_options::value<std::string>(), "Topic in bag file from which pointclouds will be read.")
    ("pclOutTopic", boost::program_options::value<std::string>(), "Topic to which the read pointclouds will be published, ie where the octomap node expects it.")
    ("octoTopic", boost::program_options::value<std::string>(), "Topic on which the octomap node publishes when pcl's are processed.")
    ("entropyService", boost::program_options::value<std::string>(), "Octomap node information service name")
    ("output,o", boost::program_options::value< std::string>(), "(optional) Output file name. If omitted calculated data will only be printed to the console.")
    ("reference", boost::program_options::value<std::string>(), "just a quick fix for missing first pointclouds")
  ;
  
  boost::program_options::positional_options_description posDesc; // for options without leading indicator
  posDesc.add("bag",-1);
  
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::command_line_parser(argc,argv).options(desc).positional(posDesc).run(), vm);
  boost::program_options::notify(vm);
  
  std::string outputPath;
  
  if(vm.count("help"))
  {
    std::cout<< desc << "\n";
    return 0;
  }
  if(!vm.count("bag"))
  {
     ROS_ERROR_STREAM("Specify a bag file.");
     return 1;
  }
  if(!vm.count("pclInTopic"))
  {
     ROS_ERROR_STREAM("Specify pclInTopic.");
     return 1;
  }
  if(!vm.count("pclOutTopic"))
  {
     ROS_ERROR_STREAM("Specify pclOutTopic.");
     return 1;
  }
  if(!vm.count("octoTopic"))
  {
     ROS_ERROR_STREAM("Specify octoTopic.");
     return 1;
  }
  if(!vm.count("entropyService"))
  {
     ROS_ERROR_STREAM("Specify entropyService.");
     return 1;
  }
  std::string outputFilePath;
  if(vm.count("output"))
  {
     outputFilePath = vm["output"].as<std::string>();
  }
  
  //dense_reconstruction::ModelCompletenessCalculator calculator;
  ROS_INFO("Starting entropy reception.");
  ROS_INFO_STREAM("Received "<<vm["bag"].as<std::vector<std::string> >().size()<<" input bags.");

  ros::Publisher pclPublisher, tfPublisher;
  ros::Subscriber octoListener;
  ros::ServiceClient entropyService;
  /*bool octomapPublished = false;
  boost::function<void (const octomap_msgs::OctomapConstPtr&)> octomapListening = 
    [&octomapPublished](const octomap_msgs::OctomapConstPtr& msg)
    {
      octomapPublished = true;
    };*/
  
  pclPublisher = n.advertise<sensor_msgs::PointCloud2>( vm["pclOutTopic"].as<std::string>(), 1 );
  tfPublisher = n.advertise<tf2_msgs::TFMessage>( "/tf", 1);
  
  // routine finishes too fast for octomap node to subscribe to the published topics - Hence wait until the connection is established before continuing
  while( pclPublisher.getNumSubscribers()==0 || tfPublisher.getNumSubscribers()==0 )
  {
    if( !n.ok() )
      return 0;
    
    ros::Duration(5).sleep();
  }
  
  octoListener = n.subscribe( vm["octoTopic"].as<std::string>(), 1, &octomapListening );
  entropyService = n.serviceClient<dense_reconstruction::ViewInformationReturn>( vm["entropyService"].as<std::string>() );
  
  std::vector<std::string> bagFiles = vm["bag"].as<std::vector<std::string> >();
  std::vector<std::string> referenceFiles;
  
  unsigned int bagFilesNr = vm["bag"].as<std::vector<std::string> >().size() - 1; // TODO remove 7 : this stuff is all just for missing pcls
  
  for( unsigned int i=0; i<vm["bag"].as<std::vector<std::string> >().size() - 1; ++i ) // TODO set equal bag input
  {
    bagFiles.push_back( vm["bag"].as<std::vector<std::string> >()[i] );
  }
  unsigned int referenceFilesMultiplicity = bagFilesNr / 1; // TODO remove
  ROS_INFO_STREAM("Calculated reference file multiplicity is: "<<referenceFilesMultiplicity); // TODO remove
  for( unsigned int i=0; i<1; ++i )
  {
    for( unsigned int j=0; j<referenceFilesMultiplicity; ++j )
    {
      referenceFiles.push_back( vm["bag"].as<std::vector<std::string> >()[bagFilesNr+i] );
    }
  }// TODO remove...
  
  std::vector< std::vector<double> > calculatedEntropies( bagFiles.size(), std::vector<double>() );
  
  std::vector<std::string> topics;
  topics.push_back("/tf");
  topics.push_back( vm["pclInTopic"].as<std::string>() );
  
  for( unsigned int i=0; i<bagFiles.size(); ++i )
  {
    ROS_INFO_STREAM("Processing bag '"<<bagFiles[i]<<"', bag nr. "<<i+1<<"/"<<bagFiles.size());
    std::vector<double> entropies; // entropies calculated for the current bag
    
    // reset octomap
    std_srvs::Empty quest;
    ros::service::call("/octomap_dense_reconstruction/reset",quest);
    
    // read bag file
    rosbag::Bag bag;
    bag.open( bagFiles[i] );
    rosbag::View viewer(bag, rosbag::TopicQuery(topics) );
    
    if( viewer.size()==0 )
    {
      ROS_ERROR_STREAM("Neither a /tf topic nor a "<<bagFiles[i]<<" topic was found in bag.");
      continue;
    }
    
    // receive entropy information
    dense_reconstruction::ViewInformationReturn viRequest;
    std::vector<geometry_msgs::Pose> poseDummy;
    poseDummy.push_back(geometry_msgs::Pose());
    viRequest.request.call.poses = poseDummy; // a pose is expected for VI calculations, so we need to currently fill it even if we only use VI that is calculated on the complete octomap
    viRequest.request.call.ray_resolution_x = 0.01;
    viRequest.request.call.ray_resolution_y = 0.01;
    std::vector<std::string> dummyNames;
    dummyNames.push_back("TotalTreeEntropy");
    viRequest.request.call.metric_names = dummyNames;
    
    bool response = entropyService.call(viRequest);
    if( response )
    {
      entropies.push_back(viRequest.response.expected_information.values[0]);
    }
    else
    {
      ROS_ERROR_STREAM("Octomap VI service reported an error");
      return 1;
    }
    ROS_INFO_STREAM("Entropies from current bag:");
    for( unsigned int what = 0; what<entropies.size(); ++what )
      std::cout<< entropies[what] << " ";
    std::cout<<std::endl;
    
    std::deque<tf2_msgs::TFMessage::Ptr> tfMessages;
    std::deque<sensor_msgs::PointCloud2::Ptr> pclMessages;
    
    BOOST_FOREACH( rosbag::MessageInstance const mInst, viewer )
    {
      tf2_msgs::TFMessage::Ptr tfMsg = mInst.instantiate<tf2_msgs::TFMessage>();
      if(tfMsg)
      {
	tfMessages.push_back(tfMsg);
      }
      sensor_msgs::PointCloud2::Ptr pclMsg = mInst.instantiate<sensor_msgs::PointCloud2>();
      if(pclMsg)
      {
	pclMessages.push_back(pclMsg);
      }
    }
    
    if( tfMessages.empty() || pclMessages.empty() )
    {
      ROS_WARN_STREAM("Either no tf or no pcl messages were found in bag.");
      continue;
    }
    
    if( pclMessages.size()!=20 ) //TODO remove
    {
      std::string referencePath_ = referenceFiles[i];
      if( boost::filesystem::exists( referencePath_+"firstCloud.ply2" )  )
      {
	ROS_INFO_STREAM("Loading extra first pointcloud from file: '"<<(referencePath_+"firstCloud.ply2")<<"', since only "<<pclMessages.size()<<" pointclouds are in bag file and therefore the first one is missing.");
	pcl::PointCloud<pcl::PointXYZRGB> pc;
	pcl::io::loadPLYFile(referencePath_+"firstCloud.ply2",pc);
	sensor_msgs::PointCloud2::Ptr pcPtr = boost::make_shared<sensor_msgs::PointCloud2>();
	pcl::toROSMsg(pc,*pcPtr);
	pcPtr->header.frame_id = "cam_pos";
	pclMessages.push_front(pcPtr);
	
	geometry_msgs::TransformStamped initialPosition;
	tf::StampedTransform initPos;
	initialPosition.header.stamp.sec = 0;
	initialPosition.header.frame_id = "dr_origin";
	initialPosition.child_frame_id = "cam_pos";
	initialPosition.transform.translation.x = 0.6;
	initialPosition.transform.translation.y = 0.0;
	initialPosition.transform.translation.z = 0.15;
	initialPosition.transform.rotation.x = 0.707107;
	initialPosition.transform.rotation.y = 0;
	initialPosition.transform.rotation.z = -0.707107;
	initialPosition.transform.rotation.w = 0;
	tf2_msgs::TFMessage::Ptr initialTf = boost::make_shared<tf2_msgs::TFMessage>();
	initialTf->transforms.push_back(initialPosition);
	tfMessages.push_front(initialTf);
      }
    } //TODO remove
      
    
    /*ros::Time newBase = ros::Time::now();
    ros::Time baseTime = (*tfMessages.front()).transforms[0].header.stamp;
    ros::Duration offset = newBase - baseTime;*/
    
    for( unsigned pclId = 0; pclId < pclMessages.size(); ++pclId )
    {
      if(!n.ok())
	    return 1;
      
      ROS_INFO_STREAM("Processing pointcloud "<<pclId+1<<"/"<<pclMessages.size()<<".");
      sensor_msgs::PointCloud2::Ptr currentPcl = pclMessages[i];
      ros::Time currentPclTime = currentPcl->header.stamp;
      
      if( tfMessages.empty() )
	ROS_ERROR_STREAM("No tf messages!!!");
      // Publish all tf messages with a timestamp older than the next pcl, convert old time to new
      while( !tfMessages.empty()  )
      {
	ROS_INFO_STREAM("Yes we here");
	if( ((*tfMessages.front()).transforms[0].header.stamp>currentPclTime) )
	{
	  break;
	}
	/*for( unsigned tfId=0; tfId<(*tfMessages.front()).transforms.size(); ++tfId )
	{
	  ros::Time newTime = (*tfMessages.front()).transforms[tfId].header.stamp + offset;
	  (*tfMessages.front()).transforms[tfId].header.stamp = newTime;
	}*/
      ROS_INFO_STREAM("Publishing tf");
	tfPublisher.publish( tfMessages.front() );
	tfMessages.pop_front();
      }
      
      //currentPcl->header.stamp = currentPclTime + offset;
      octomapPublished = false;
      pclPublisher.publish(currentPcl);
      
      // wait for octomap to integrate the message
      do
      {
	ros::Duration(0.1).sleep(); 
	ros::spinOnce();
	
	if(!n.ok())
	  return 1;
	
      } while(!octomapPublished);
      
      ROS_INFO_STREAM("Octomap published - Attempting to receive entropy information");
	
      // receive entropy information
      
      bool response = entropyService.call(viRequest);
      if( response )
      {
	entropies.push_back(viRequest.response.expected_information.values[0]);
      }
      else
      {
	ROS_ERROR_STREAM("Octomap VI service reported an error");
	return 1;
      }
      ROS_INFO_STREAM("Entropies from current bag:");
      for( unsigned int what = 0; what<entropies.size(); ++what )
	std::cout<< entropies[what] << " ";
      std::cout<<std::endl;
    }
        
    calculatedEntropies[i] = entropies;
  }
  
  
  if( !n.ok() )
    return 0;
  
  std::stringstream outputText;
  for( unsigned int i=0; i<calculatedEntropies.size(); ++i )
  {
    outputText << "Entropies for "<<bagFiles[i]<<":\n";
    for( unsigned int e=0; e<calculatedEntropies[i].size(); ++e )
    {
      outputText << calculatedEntropies[i][e] << " ";
    }
    outputText << std::endl;
  }
  
  // output file if an output file name was given
  if(outputPath!="")
  {
    ROS_INFO_STREAM("Writing calculated values to file");
    std::ofstream outputFile;
    outputFile.open( outputPath.c_str() );
    outputFile << outputText.str();
    outputFile.close();
  }
  
  ROS_INFO_STREAM("Entropy calculation has finished.");
  
  std::cout << "\n\n\n";
  std::cout << outputText.str();
  
  return 0;
} 
