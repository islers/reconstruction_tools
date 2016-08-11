/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of reconstruction_tools, a ROS package for...well,

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

#pragma once

#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>

#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>

namespace reconstruction_tools
{
  
  /*! Class that calculates how much of a given model is covered by pointclouds by comparing pcl data from a bag
   * with point in a .ply reference file. Pointclouds in the given bag topic are accumulated and at each accumulation
   * step a completeness percentage is being computed. The completeness percentage is defined as the number of points
   * in the reference model that have at least one neighbour in the accumulated pointcloud that is closer than a 
   * given registration distance over the total number of points in the reference, i.e. registeredPoints/totalNrOfPoints.
   */
  class ModelCompletenessCalculator
  {
  public:
    /*! Constructor
     */
    ModelCompletenessCalculator(){};
    
    /*! Loads pcl data from a bag and calculates the completeness values. Ground truth and register distance must have been set before.
     * @param path Path to the bag file.
     */
    void processInputBag( std::string path );
    
    /*! Loads pcl data from a ply file and calculates the completeness values. Ground truth and register distance must have been set before.
     *
     * @param path Path to the ply file.
     */
    void processInputPly( std::string path );
    
    /*! Loads a pointcloud from a ply file that is to be used as ground truth.
     * @param path Path to the ply file.
     */
    bool setGroundTruth( std::string path );
    
    /*! Returns any possibly previously calculated completeness vector.
     * @param completeness (output) Output vector with the completeness percentages.
     */
    void getCompleteness( std::vector<double>& completeness );
    
    /*! Sets the registration distance.
     * @param distance The distance [m]
     */
    void setRegistrationDistance( double distance );
    
    /*! Sets the target frame.
     * @param targetFrame The target frame.
     */
    void setTargetFrame( std::string targetFrame );
    
    /*! Sets the pcl topic.
     * @param pclTopic The pcl topic.
     */
    void setPclTopic( std::string pclTopic );
    
    /*! Returns the completeness percentage for a given pointcloud when compared to the reference pointcloud.
     * @param toCompare Pointcloud for which the completeness percentage is to be computed.
     * @return The completeness percentage.
     */
    double calculateCompleteness( pcl::PointCloud<pcl::PointXYZRGB>& toCompare );
    
  private:
    //std::string path_;
    
    rosbag::Bag bag_;
    double registerDistance_;
    
    std::vector<double> completeness_;
    
    std::deque<sensor_msgs::PointCloud2::Ptr> bagContent_;
    std::queue<tf::StampedTransform> tfFifo_;
    
    std::string targetFrame_; //! Frame to which pcl data is to be transformed
    std::string pclTopic_; //! Topic name in bag from which pcl data is read
    std::string referencePath_; //! Path to the last added ground truth.
    
    pcl::PointCloud<pcl::PointXYZRGB> groundTruth_; // Ground truth data.
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_; // Retrieved data.
  };
  
}