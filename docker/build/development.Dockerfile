FROM osrf/ros:humble-desktop AS base

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
   ros-humble-ament* \
   ros-humble-rqt* \
   ros-humble-navigation2 \
   ros-humble-nav2-bringup \
   ros-humble-rmw-cyclonedds-cpp \
   ros-humble-tf-transformations \
   ros-humble-tf2-tools \
   ros-humble-turtle-tf2-py \
   ros-humble-pluginlib \
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

# Clone and compile vda5050_connector
RUN mkdir -p /home/docker/.ssh && ssh-keyscan -H github.com > /home/docker/.ssh/known_hosts
RUN --mount=type=ssh,mode=0666 /bin/bash -c \
   'source /opt/ros/humble/setup.bash \
   && mkdir -p src \
   && git clone --branch humble-devel https://github.com/inorbit-ai/ros_amr_interop.git \
   && mv ros_amr_interop/vda5050_connector ./src/vda5050_connector \
   && mv ros_amr_interop/vda5050_msgs ./src/vda5050_msgs \
   && mv ros_amr_interop/vda5050_serializer ./src/vda5050_serializer \
   && rm -rf ros_amr_interop \
   && /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"'

# Use CycloneDDS (see https://github.com/ROBOTIS-GIT/turtlebot3/issues/884#issuecomment-1239245562)
# and add ROS and workspace overlays to docker user
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/docker/.bashrc \
   && echo "source /opt/ros/humble/setup.bash" >> /home/docker/.bashrc \
   && echo "source /home/docker/dev_ws/install/setup.bash" >> /home/docker/.bashrc \
   && rosdep update
