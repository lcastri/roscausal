#!/bin/bash

set -e

# setup environment
source "$HOME/.bashrc"

echo " "
echo "###"
echo "### This is the ROSCausal container!"
echo "###"
echo " "

{

  echo "Container is now running."
  echo " "
   source /opt/ros/noetic/setup.bash
   cd ~/ros_ws
   catkin build
   source ~/ros_ws/devel/setup.bash
   echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
   exec "/bin/bash"

} || {

  echo "Container failed."
  exec "$@"

}
