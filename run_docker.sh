#!/bin/bash

container_name=ROSCausal
image_name=roscausal-docker
host_folder=$(pwd)/shared
container_folder=/root/shared

xhost +local:

echo " "
echo "Running ${container_name} container..."
docker run --name ${container_name} -it --rm  -e DISPLAY=$DISPLAY -e NVIDIA_DRIVER_CAPABILITIES=all -e QT_X11_NO_MITSHM=1 -v $HOME/.Xauthority:/root/.Xauthority:rw -v /tmp/.X11-unix:/tmp/.X11-unix -v $host_folder:$container_folder --privileged --gpus all --network host ${image_name}

echo " "
echo "Closing ${container_name} container..."
xhost -local: