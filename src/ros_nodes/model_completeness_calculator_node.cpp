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

 
#include "reconstruction_tools/model_completeness_calculator.h"

#include <boost/program_options.hpp>

#include <fstream>

using namespace std;

/*! Calculates completeness for a given bag/reference file pair
 * @param bagPath Path to the bag.
 * @param referencePath Path to the reference file
 * @param pclTopic Pointcloud topic within the bag
 * @param targetFrame Frame to which pointclouds shall be transferred. For targetFrame="", no transformation is carried out
 * @param maxRegistrationDistance Max. distance between points from the bag and the reference to be registered.
 * @param completeness (output) Output vector with completeness for each (accumulated) pcl from bag.
 * @param usePlyMethod Whether instead of the bag, the ply method shall be used (for ply inputs).
 */
void calculateCompleteness( std::string bagPath, 
			    std::string referencePath, 
			    std::string pclTopic,
			    std::string targetFrame,
			    double maxRegistrationDistance,
			    std::vector<double>& completeness, bool usePlyMethod=false )
{
  ROS_INFO_STREAM("Processing bag file "<<bagPath);
  ROS_INFO_STREAM("Used reference file is "<<referencePath);
  reconstruction_tools::ModelCompletenessCalculator calculator;
  calculator.setGroundTruth(referencePath);
  calculator.setRegistrationDistance(maxRegistrationDistance);
  calculator.setTargetFrame(targetFrame);
  calculator.setPclTopic(pclTopic);
  
  if(!usePlyMethod)
    calculator.processInputBag(bagPath);
  else
    calculator.processInputPly(bagPath);
  
  calculator.getCompleteness(completeness);
  
  ROS_INFO_STREAM("Calculated "<<completeness.size()<<" completeness percentages.");
}

/*! Same as calculateCompleteness(...) but for an array of bag files and reference files
 */
void calculateCompletenessOnArray( std::vector<std::string>& bagPaths, 
			    std::vector<std::string>& referencePaths, 
			    std::string pclTopic,
			    std::string targetFrame,
			    double maxRegistrationDistance,
			    std::vector<std::vector<double> >& completenesses, bool usePlyMethod=false )
{
  if( bagPaths.size()!=referencePaths.size() )
  {
    ROS_ERROR_STREAM("model_completeness_calculator node:model_completeness_calculator: given bag and reference file arrays are not of same size");
    return;
  }
  
  for( unsigned int i=0; i<bagPaths.size(); ++i )
  {
    ros::NodeHandle n;
    if( !n.ok() )
      return;
    std::vector<double> completeness;
    calculateCompleteness(bagPaths[i], referencePaths[i],pclTopic,targetFrame,maxRegistrationDistance,completeness,usePlyMethod);
    
    completenesses.push_back( completeness );
    
    //if(i>2) // TODO REMOVE!
    //  break;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "model_completeness_calculator");
  ros::NodeHandle n;
  
  boost::program_options::options_description desc("This node reads pcl data from a bag file and compares it to a ground truth pcl: The calculated metric is the quotient of points in the reference for which a neighbour point closer than a given distance was found in the pcl data over the total number of points in the ground truth pcl, i.e. registeredPoints/pointsInGroundTruth. The pointclouds in the bag are accumulated and a completeness value is being computed for each step during the accumulation.\n\nExample 1: model_completeness_calculator mybag.bag -t /topic -f /target_frame -r groundtruth.ply -d 0.01 -o data.txt\nExample 2: model_completeness_calculator mybag.bag groundtruth.ply -t /topic -f /target_frame -d 0.01 -o data.txt\nExample 3: model_completeness_calculator *.bag *.ply -t /topic -f /target_frame -d 0.01 -o data.txt\nExample 4: model_completeness_calculator bag1.bag bag2.bag bag3.bag reference.ply -bag_file_nr 3 -t /topic -f /target_frame -d 0.01 -o data.txt\n\n");
  
  desc.add_options()
    ("help,h", "Prints this help.")
    ("bag,b", boost::program_options::value< std::vector<std::string > >(), "Path to the input bag from which pcl data is to be read. If you want to process more than one file, omit the qualifier and directly pass the list of bag files followed by a list of respective reference files of the same size. The total number of passed files must thus be even. The first half will be interpreted as bag files, the second half as ply files.")
    ("topic,t", boost::program_options::value<std::string>(), "Topic in bag file from which pointclouds will be read.")
    ("target_frame,f", boost::program_options::value<std::string>(), "(optional) Target frame to which the pointclouds will be transformed, necessitates an available /tf topic. No time-out exits, ie each transformation available from /tf will be used until a new one is available. If not defined, no transformation is carried out")
    ("registration_distance,d", boost::program_options::value<double>(), "Maximal distance between points from the bag and the reference to be considered as a correspondence for completeness calculation. [m]")
    ("reference,r", boost::program_options::value< std::string >(), "Path to the .ply reference file. Refer to the explanation for the bag file if you want to process more than one file.")
    ("output,o", boost::program_options::value< std::string>(), "(optional) Output file name. If omitted calculated data will only be printed to the console.")
    ("bag_file_nr", boost::program_options::value<unsigned int>(), "(optional) Total number of bag files given as arguments. If a reference file is to be used for more than one file, provide this parameter. The following must hold: b/(n-b)=m, where n is the number of unnamed argument files passed, b is the bag_file_nr and m is the number each reference file is used. The order of arguments must be such that first all m bag files are listed that belong to the first reference file, second all m bag files that belong to the second reference file and so on. If this parameter is set, any -reference argument will be ignored.")
    ("ref_file_nr", boost::program_options::value<unsigned int>(), "(optional) Same as 'bag_file_nr' but instead of the number of bag files, define the number of reference file.")
    ("ply", "(optional) If this option is set, it will be assumed that instead of bag files, the input are ply files.")
  ;
  
  boost::program_options::positional_options_description posDesc; // for options without leading indicator
  posDesc.add("bag",-1);
  
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::command_line_parser(argc,argv).options(desc).positional(posDesc).run(), vm);
  boost::program_options::notify(vm);
  
  std::string targetFrameName, outputPath, referencePath, topicName;
  double registrationDistanceM;
  unsigned int bagFileNr = 0, refFileNr=0;
  
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
  else if( vm["bag"].as< std::vector<std::string> >().size()==1 && !vm.count("reference") )
  {
    ROS_ERROR_STREAM("Define a reference file.");
    return 1;
  }
  if( vm.count("reference") && vm["bag"].as< std::vector<std::string> >().size()==1 )
    referencePath = vm["reference"].as<std::string>();
  
  if( !vm.count("topic") )
  {
    ROS_ERROR_STREAM("Define a pointcloud topic.");
    return 1;
  }
  else
    topicName = vm["topic"].as<std::string>();
  
  if( vm.count("target_frame") )
  {
    targetFrameName = vm["target_frame"].as<std::string>();
    ROS_INFO_STREAM("Target frame: '"<<targetFrameName<<"' defined.");
  }
  else
    ROS_INFO_STREAM("No target frame defined, pointclouds will not be transferred.");
  if( vm.count("output") )
  {
    outputPath = vm["output"].as<std::string>();
  }
  if( !vm.count("registration_distance") )
  {
    ROS_ERROR_STREAM("No registration distance defined (-d option)");
    return 1;
  }
  else
  {
    registrationDistanceM = vm["registration_distance"].as<double>();
  }
  if( vm.count("bag_file_nr") )
  {
    bagFileNr = vm["bag_file_nr"].as<unsigned int>();
  }
  if( vm.count("ref_file_nr") )
  {
    refFileNr = vm["ref_file_nr"].as<unsigned int>();
  }
  
  //dense_reconstruction::ModelCompletenessCalculator calculator;
  ROS_INFO("Starting model completeness calculation.");
  std::vector<std::vector<double> > completenesses;
  
  
  if( vm["bag"].as<std::vector<std::string> >().size()==1 )
  { // standard mode: one bag file through -b and one reference through -r
    std::string bagFile = vm["bag"].as<std::vector<std::string> >()[0];
    
    std::vector<double> completeness;
    calculateCompleteness(bagFile, referencePath, topicName, targetFrameName, registrationDistanceM, completeness);
    completenesses.push_back(completeness);
  }
  else if( bagFileNr!=0 || refFileNr!=0 )
  { // multiple bag files with multiple reference files where each reference filed is used more than once
    // test if file numbers match
    unsigned int fileArgumentsNr = vm["bag"].as<std::vector<std::string> >().size();
    
    if( refFileNr!=0 )
    {
      bagFileNr = fileArgumentsNr - refFileNr;
    }
    
    double refFileMultiplicity = bagFileNr/(fileArgumentsNr-bagFileNr);
    double multiplicityRemainder = refFileMultiplicity - (int)refFileMultiplicity;
    if( multiplicityRemainder!=0 )
    {
      ROS_ERROR_STREAM("bag_file_nr parameter was set but the number of files given as arguments and the given bag_file_nr do not match: bag_file_nr is not a multiple of the number of reference files.");
      return 1;
    }
    
    // build file vectors
    std::vector<std::string> bagPaths;
    std::vector<std::string> referencePaths;
    for( unsigned int i=0; i<(fileArgumentsNr-bagFileNr); ++i )
    {
      for( unsigned int r=0; r<refFileMultiplicity; ++r )
      {
	bagPaths.push_back( vm["bag"].as<std::vector<std::string> >()[i*refFileMultiplicity + r] );
	referencePaths.push_back( vm["bag"].as<std::vector<std::string> >()[bagFileNr + i] );
      }
    }
    
    calculateCompletenessOnArray(bagPaths, referencePaths, topicName, targetFrameName, registrationDistanceM, completenesses, vm.count("ply") );
  }
  else
  { // multiple input files, first half are bag files, second half are the reference ply files
    
    unsigned int fileArgumentsNr = vm["bag"].as<std::vector<std::string> >().size();
    
    // number of arguments must be even
    if( fileArgumentsNr%2 != 0 )
    {
      ROS_ERROR_STREAM("A list of files was given as argument, but -bag_file_nr was not set and the number of arguments is uneven. Recheck the list.");
      return 1;
    }
    
    // build file vectors
    std::vector<std::string> bagPaths;
    std::vector<std::string> referencePaths;
    for( unsigned int i=0; i<(fileArgumentsNr/2); ++i )
    {
      bagPaths.push_back( vm["bag"].as<std::vector<std::string> >()[i] );
      referencePaths.push_back( vm["bag"].as<std::vector<std::string> >()[fileArgumentsNr/2 + i] );
    }
    
    calculateCompletenessOnArray(bagPaths, referencePaths, topicName, targetFrameName, registrationDistanceM, completenesses);
  }
  
  if( !n.ok() )
    return 0;
  
  ROS_INFO_STREAM("Completeness percentage computations are finished.");
  
  std::stringstream outputText;
  outputText << "Completeness percentage results were:\n----------------\n"<<"Used registration distance: "<<registrationDistanceM<<"m.\n\n";
  // Compute output text
  for( unsigned int i=0; i<completenesses.size(); ++i )
  {
    outputText << vm["bag"].as<std::vector<std::string> >()[i] << std::endl;
    for( unsigned int j=0; j<completenesses[i].size(); ++j )
    {
      outputText << completenesses[i][j] << " ";
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
  
  std::cout << "\n\n\n";
  std::cout << outputText.str();
  
  return 0;
} 
