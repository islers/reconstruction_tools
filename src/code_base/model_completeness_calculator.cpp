#include "reconstruction_tools/model_completeness_calculator.h"
#include "ros_tools/pcl_accumulator.hpp"
//#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <boost/filesystem.hpp>

namespace reconstruction_tools
{
  
  void ModelCompletenessCalculator::processInputBag( std::string path )
  {
    bag_.open( path, rosbag::bagmode::Read );
    ros::NodeHandle n;
    
    if( pclTopic_=="" )
    {
      ROS_ERROR_STREAM("ModelCompletenessCalculator::processInputBag: Error: No pcl topic defined.");
      return;
    }
    std::vector<std::string> topics;
    topics.push_back(pclTopic_/*"remode/pointcloud_single"*/);
    topics.push_back("/tf");
    rosbag::View viewer( bag_, rosbag::TopicQuery(topics) );
    
    
    pcl::PointCloud<pcl::PointXYZRGB> pc;
    pcl::PointCloud<pcl::PointXYZRGB> icpFilteredNew;
    
    ros_tools::PclAccumulator accumulator;
    ros_tools::PclAccumulator::IcpSetting icpSetting = ros_tools::PclAccumulator::NO_ICP; // TODO: This should be offered as setting to the user -> first test if pcl ICP works as expected
    
    BOOST_FOREACH( rosbag::MessageInstance const m, viewer )
    {
      sensor_msgs::PointCloud2::Ptr msg = m.instantiate<sensor_msgs::PointCloud2>();
      if(msg)
	bagContent_.push_back(msg);
      else
      {
	tf2_msgs::TFMessage::Ptr tfMsg = m.instantiate<tf2_msgs::TFMessage>();
	if(tfMsg)
	{
	  for( unsigned int transformId=0; transformId<tfMsg->transforms.size(); ++transformId )
	  {
	    tf::StampedTransform tfTransform;
	    tf::transformStampedMsgToTF(tfMsg->transforms[transformId],tfTransform);
	    tfFifo_.push(tfTransform);
	  }
	}
      }
    }
    
    //ROS_ERROR_STREAM("processInputBag:"<<path<<" - Found "<<bagContent_.size()<<" pcls");
    tf::Transformer tfTracker;
    
    //TODO: remove this again... just to make sure a first transform is available
    if( bagContent_.size()!=20 )
    {
      if( boost::filesystem::exists( referencePath_+"firstCloud.ply2" )  )
      {
	pcl::PointCloud<pcl::PointXYZRGB> pc;
	pcl::io::loadPLYFile(referencePath_+"firstCloud.ply2",pc);
	sensor_msgs::PointCloud2::Ptr pcPtr = boost::make_shared<sensor_msgs::PointCloud2>();
	pcl::toROSMsg(pc,*pcPtr);
	pcPtr->header.frame_id = "cam_pos";
	bagContent_.push_front(pcPtr);
      }
    }
    else
    { // code to save the first pointcloud to a file
      /*ROS_INFO_STREAM("Saving first cloud file: "<<(referencePath_+"firstCloud.ply2"));
      pcl::PointCloud<pcl::PointXYZRGB> pc;
      pcl::fromROSMsg(*bagContent_[0], pc);
      std::vector<int> placeholder;
      pcl::removeNaNFromPointCloud(pc,pc,placeholder);
      
      pcl::io::savePLYFile(referencePath_+"firstCloud.ply2",pc,false);*/
    }
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
    
    tf::transformStampedMsgToTF(initialPosition,initPos);
    tfTracker.setTransform( initPos );
    // TODO until here...
    
    for( unsigned int i=0; i<bagContent_.size(); ++i )
    {
      if(i==34)
	continue; //hack
      ROS_INFO_STREAM("Processing pointcloud "<<i+1<<"/"<<bagContent_.size()<<".");
      sensor_msgs::PointCloud2::Ptr currentPcl = bagContent_[i];
      ros::Time currentPclTime = currentPcl->header.stamp;
      
      if(!n.ok())
	return;
      
      while( !tfFifo_.empty() && (tfFifo_.front().stamp_<=currentPclTime) )
      {
	tfTracker.setTransform( tfFifo_.front() );
	tfFifo_.pop();
      }
      std::string targetFrame = targetFrame_;//"/dr_origin";
      if( targetFrame!="" && tfTracker.canTransform( targetFrame, currentPcl->header.frame_id, ros::Time(0) ) )
      {
	tf::StampedTransform tfToApply;
	tfTracker.lookupTransform( targetFrame, currentPcl->header.frame_id, ros::Time(0), tfToApply );
	sensor_msgs::PointCloud2::ConstPtr currentPclConst = boost::const_pointer_cast<const sensor_msgs::PointCloud2>(currentPcl);
	accumulator.pushPclFromRosAndTransform( currentPclConst, tfToApply, icpSetting );
      }
      else if( targetFrame!="" )
      {
	ROS_ERROR_STREAM("Couldn't transform pointcloud to target frame '"<<targetFrame<<"'.");
	return;
      }
      else
      {
	sensor_msgs::PointCloud2::ConstPtr currentPclConst = boost::const_pointer_cast<const sensor_msgs::PointCloud2>(currentPcl);
	accumulator.pushPclFromRos( currentPclConst, icpSetting );
      }
      /*
      pcl::fromROSMsg(*bagContent_[i], pc);
      if(i=0)
      {
	cloud_ += pc;
      }
      else
      {
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource( pc.makeShared() );
        icp.setInputTarget( cloud_.makeShared() );
        icp.setMaxCorrespondenceDistance( 0.0005 );
	icp.align( icpFilteredNew );
	
	if( icp.hasConverged() )
	{
	  cloud_+=icpFilteredNew;
	}
	else
	{
	  icpFilteredNew=pc;
	  cloud_+=pc;
	}
      }*/
      
      cloud_ = accumulator.currentCloud();
      
      completeness_.push_back( calculateCompleteness(*cloud_) );
      
      //ROS_INFO_STREAM("Current completeness percentages: "<<completeness_);
      std::cout<<"\nCurrent completeness percentages: ";
      for(unsigned int it=0; it<completeness_.size(); ++it )
	std::cout<<completeness_[it]<<" ";
      std::cout<<"\n";
            
      /*std::string path = path_+"/pclStep"+boost::to_string(i)+".ply";
      pcl::io::savePLYFile(path,icpFilteredNew,false);
      std::string total = path_+"/pclIntegrated"+boost::to_string(i)+".ply";
      pcl::io::savePLYFile(total,*cloud_);*/
      
      
    }
    
  }
  
  void ModelCompletenessCalculator::processInputPly( std::string path )
  {
    pcl::PointCloud<pcl::PointXYZRGB> plyCloud;
    bool success = ( pcl::io::loadPLYFile( path, plyCloud )==0 );
    
    (*cloud_) = plyCloud;
    
    if( !success )
    {
      ROS_WARN_STREAM("ModelCompletenessCalculator::processInputPly::An error occured when loading ply file.");
      return;
    }
    
    completeness_.push_back( calculateCompleteness(*cloud_) );
    
    std::cout<<"\nCurrent completeness percentages: ";
    for(unsigned int it=0; it<completeness_.size(); ++it )
      std::cout<<completeness_[it]<<" ";
    std::cout<<"\n";
    
  }
  
  bool ModelCompletenessCalculator::setGroundTruth( std::string path )
  {
    referencePath_ = path;
    bool success = ( pcl::io::loadPLYFile( path, groundTruth_ )==0 );
    
    /*if( success )
    {
      ROS_INFO_STREAM("loadPLYFile probably succeeded");
      
      ROS_INFO_STREAM("Ground Truth size: "<<groundTruth_.points.size());
    }
    else
      ROS_INFO_STREAM("loadPLY probably didn't succeed");*/
    
    return success;
  }
  
  void ModelCompletenessCalculator::getCompleteness( std::vector<double>& completeness )
  {
    completeness = completeness_;
  }
  
  void ModelCompletenessCalculator::setRegistrationDistance( double distance )
  {
    registerDistance_ = distance;
  }
  
  void ModelCompletenessCalculator::setTargetFrame( std::string targetFrame )
  {
    targetFrame_ = targetFrame;
  }
  
  void ModelCompletenessCalculator::setPclTopic( std::string pclTopic )
  {
    pclTopic_ = pclTopic;
  }
  
  double ModelCompletenessCalculator::calculateCompleteness( pcl::PointCloud<pcl::PointXYZRGB>& toCompare )
  {
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    
    if( toCompare.size()==0 )
    {
      ROS_INFO_STREAM("One input cloud was empty and will be skipped.");
      return -1;
    }
    kdtree.setInputCloud( toCompare.makeShared() );
    
    unsigned int pointsInGroundTruth = groundTruth_.points.size();
    unsigned int registeredPoints = 0;
    
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    
    unsigned int counter=0;
    
    ros::NodeHandle n;
    
    for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = groundTruth_.begin(); it != groundTruth_.end(); ++it, ++counter )
    {
      pcl::PointXYZRGB searchPoint = *it;
      //point3d point(it->x, it->y, it->z);
      
      if(counter%5000==0)
      {
	ROS_INFO_STREAM("Processing point "<<counter<<"/"<<pointsInGroundTruth<<", ie "<<counter/(double)pointsInGroundTruth*100<<"% processed.");
      }
      
      if( kdtree.radiusSearch(searchPoint, registerDistance_, pointIdxRadiusSearch, pointRadiusSquaredDistance )>0 )
      {
	++registeredPoints;
      }
      
      if(!n.ok())
	return -1.0;
    }
    
    return (double)registeredPoints/(double)pointsInGroundTruth;
  }
  
}