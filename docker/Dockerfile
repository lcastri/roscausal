FROM osrf/ros:noetic-desktop-full

# Install essential packages
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \ 
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    locales lsb-release build-essential software-properties-common gnupg curl wget \
    apt-transport-https git tmux terminator nano htop net-tools iputils-ping sshpass iproute2 python3-pip \
    xserver-xorg-video-dummy x11-* xinit x11vnc xterm xvfb xauth xorg python3-catkin-tools ros-noetic-move-base-msgs \
    && rm -rf /var/lib/apt/lists/*

# Create a new workspace
RUN mkdir -p ~/ros_ws
WORKDIR /root/ros_ws

# Install tmux and TMuLE
RUN DEBIAN_FRONTEND=noninteractive pip install tmule
    
# install fpcmci
RUN DEBIAN_FRONTEND=noninteractive pip install --upgrade setuptools wheel
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \ 
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends default-jdk
ENV JAVA_HOME "/lib/jvm/java-11-openjdk-amd64/bin/java"
RUN cd ~/ros_ws && \
    git clone -b v1.4 https://github.com/pwollstadt/IDTxl.git
RUN cd ~/ros_ws/IDTxl && pip install .
RUN DEBIAN_FRONTEND=noninteractive pip install fpcmci --use-pep517

# Installing pkgs for postprocessing
RUN DEBIAN_FRONTEND=noninteractive pip install shapely

# Clone and install roscausal
RUN mkdir -p ~/ros_ws/src/ && \ 
    cd ~/ros_ws/src/ && \
    git clone -b darko-intweek2024 https://github.com/lcastri/roscausal.git

# Copy the entrypoint.sh script into the image and give execute permissions to the script
COPY docker/entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh

# Create a shared folder with the host machine
RUN mkdir -p ~/shared

# Set the entrypoint to run the script
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]