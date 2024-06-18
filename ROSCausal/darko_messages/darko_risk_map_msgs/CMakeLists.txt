cmake_minimum_required(VERSION 3.0.2)
project(darko_risk_map_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  nav_msgs
)


add_message_files(
   FILES
   object.msg
   Objects.msg
   PriorList.msg
   prior_object.msg
   
)
generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs
  
)


catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs nav_msgs
)
