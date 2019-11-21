cmake_minimum_required(VERSION 2.8.3)
project(laser_scan_base)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rosconsole roscpp sensor_msgs message_generation rospy laser_geometry std_msgs pcl_ros tf)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES laser_scan_base
    # CATKIN_DEPENDS other_catkin_pkg
    # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(get_sick_scan src/get_sick_scan.cpp)
target_link_libraries(get_sick_scan ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(sick_scan2pcd src/sick_scan2pcd.cpp)
target_link_libraries(sick_scan2pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pcd_sac_segmentation src/pcd_sac_segmentation.cpp)
target_link_libraries(pcd_sac_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pcd_sac_segmentation2 src/pcd_sac_segmentation2.cpp)
target_link_libraries(pcd_sac_segmentation2 ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pcd_segment src/pcd_segment.cpp)
target_link_libraries(pcd_segment ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
