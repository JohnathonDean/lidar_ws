cmake_minimum_required(VERSION 3.5)
project(lidar_data_filter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++11)
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
    # LIBRARIES lidar_data_filter
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

add_executable(get_lidar_pcd src/get_lidar_pcd.cpp)
target_link_libraries(get_lidar_pcd ${catkin_LIBRARIES})

add_executable(get_lidar_scan src/get_lidar_scan.cpp)
target_link_libraries(get_lidar_scan ${catkin_LIBRARIES})

add_executable(pcd_intensities_filter src/pcd_intensities_filter.cpp)
target_link_libraries(pcd_intensities_filter ${catkin_LIBRARIES})

add_executable(scan2pcd src/scan2pcd.cpp)
target_link_libraries(scan2pcd ${catkin_LIBRARIES})

add_executable(cloud_segment src/cloud_segment.cpp)
target_link_libraries(cloud_segment ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ransac_filter src/ransac_filter.cpp)
target_link_libraries(ransac_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(cloud_sac_segmentation src/cloud_sac_segmentation.cpp)
target_link_libraries(cloud_sac_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(cloud_sac_segmentation2 src/cloud_sac_segmentation2.cpp)
target_link_libraries(cloud_sac_segmentation2 ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
