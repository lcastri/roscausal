cmake_minimum_required(VERSION 2.8.3)
project(roscausal_robot)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

# Set CMAKE_PREFIX_PATH
# set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /root/ros_ws/src/pedsim_ros/pedsim_msgs)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  roscausal_msgs
  move_base_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################
# Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   RobotState.msg
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gzb_vel_plugin
#  CATKIN_DEPENDS geometry_msgs message_generation rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


#############
## Install ##
#############


