cmake_minimum_required(VERSION 2.8.3)
project(straight_line_fitting)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs message_generation tf)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

add_message_files(FILES
  charactor.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES straight_line_fitting
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
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

add_library(ransac_fitline SHARED src/ransac_fitline.cpp)
add_library(ransac SHARED src/ransac.cpp)

add_executable(get_position src/get_position.cpp src/ransac.cpp src/ransac_fitline.cpp)
target_link_libraries(get_position ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

