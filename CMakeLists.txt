cmake_minimum_required(VERSION 2.8.3)
project(reconstruction_tools)

#list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}") # use c++03 since current PCL build is not compatible with c++11 builds

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  sensor_msgs
  cv_bridge
  geometry_msgs
  roscpp
  tf
  control_msgs
  std_msgs
  rosbag
  ros_tools_cpp03
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system )

find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs cv_bridge control_msgs tf std_msgs rosbag ros_tools_cpp03
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(model_completeness_calculator
  src/ros_nodes/model_completeness_calculator_node.cpp
  src/code_base/model_completeness_calculator.cpp
)

target_link_libraries(model_completeness_calculator
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ros_tools_cpp03
  ${ros_tools_cpp03_LIBRARIES}
)

add_dependencies(model_completeness_calculator
 ${catkin_EXPORTED_TARGETS}
)

add_executable(entropy_from_pcl_calculator
  src/ros_nodes/entropy_from_pcl_calculator_node.cpp
)

target_link_libraries(entropy_from_pcl_calculator
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ros_tools_cpp03
  ${ros_tools_cpp03_LIBRARIES}
)

add_dependencies(entropy_from_pcl_calculator
 ${catkin_EXPORTED_TARGETS}
)