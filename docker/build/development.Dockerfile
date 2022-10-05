FROM osrf/ros:galactic-desktop AS base

ENV container docker
ENV DEBIAN_FRONTEND=noninteractive

# Install / update tools
RUN apt-get update \
   && apt-get install --no-install-recommends -y  \
   apt-transport-https \
   apt-utils \
   bash-completion \
   curl \
   git \
   libffi-dev \
   psmisc \
   python3-dev \
   python3-pip \
   software-properties-common \
   sudo \
   tmux \
   unzip \
   vim \
   xterm \
   && rm -r /var/lib/apt/lists/*

# Install bootstrap tools
RUN apt-get update \
   && apt-get install --no-install-recommends -y \
   python3-rosdep \
   python3-rosinstall \
   && rm -r /var/lib/apt/lists/*

# Additional packages
RUN apt-get update \
   && apt-get install --no-install-recommends -y \
   mosquitto-clients \
   python3-argcomplete \
   python3-colcon-common-extensions \
   python3-pytest-mock \
   python3-rosdep2 \
   python3-vcstool \
   libyaml-cpp-dev \
   ros-galactic-ament* \
   ros-galactic-rqt* \
   ros-galactic-navigation2 \
   ros-galactic-nav2-bringup \
   ros-galactic-tf-transformations \
   ros-galactic-tf2-tools \
   ros-galactic-turtle-tf2-py \
   ros-galactic-pluginlib \
   && apt dist-upgrade -y \
   && rm -r /var/lib/apt/lists/*

# Install pip dependencies
RUN pip3 install --no-cache \
   transforms3d \
   paho-mqtt

# Add user `docker`, set password to `docker` and add it to group `sudo`
RUN useradd -m docker -s /bin/bash && echo "docker:docker" | chpasswd \
   && adduser docker sudo \
   && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER docker

WORKDIR /home/docker/dev_ws

# Clone and compile vda5050_msgs
RUN /bin/bash -c \
   'mkdir -p src && source /opt/ros/galactic/setup.bash \
   && git clone --branch ros2-vda5050-v2 https://github.com/ipa320/vda5050_msgs.git \
   && mv vda5050_msgs/vda5050_msgs ./src/vda5050_msgs \
   && rm -rf vda5050_msgs \
   && /bin/bash -c "source /opt/ros/galactic/setup.bash && colcon build"'

# Clone and compile vda5050_connector
RUN mkdir -p /home/docker/.ssh && ssh-keyscan -H github.com > /home/docker/.ssh/known_hosts
RUN --mount=type=ssh,mode=0666 /bin/bash -c \
   'source /opt/ros/galactic/setup.bash \
   && git clone --branch galactic-devel git@github.com:inorbit-ai/ros_amr_interop.git \
   && mv ros_amr_interop/vda5050_connector ./src/vda5050_connector \
   && /bin/bash -c "source /opt/ros/galactic/setup.bash && colcon build"'

# Add ROS and workspace overlays to docker user
RUN echo "source /opt/ros/galactic/setup.bash" >> /home/docker/.bashrc \
   && echo "source /home/docker/dev_ws/install/setup.bash" >> /home/docker/.bashrc \
   && rosdep update
