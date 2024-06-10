cmake_minimum_required(VERSION 3.0.2)
project(darko_message_visualization)

find_package(catkin REQUIRED COMPONENTS
  darko_perception_msgs
  jsk_recognition_msgs
  rospy
)

# catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS darko_perception_msgs jsk_recognition_msgs rospy
)

catkin_install_python(PROGRAMS
  scripts/visualize_scene_objects.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
