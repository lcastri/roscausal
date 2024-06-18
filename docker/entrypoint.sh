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
  echo "function rc-start(){  tmule -c ~/ros_ws/src/ROSCausal/roscausal_tmule/tmule/roscausal.yaml -W 3 launch ; }" >> ~/.bashrc
  echo "function rc-stop(){  tmule -c ~/ros_ws/src/ROSCausal/roscausal_tmule/tmule/roscausal.yaml terminate ; }" >> ~/.bashrc
  echo "function rc-show(){  tmux a -t roscausal ; }" >> ~/.bashrc

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
