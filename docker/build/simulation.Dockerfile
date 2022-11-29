FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM=1

# Install / update tools
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
    curl \
    git \
    gnupg2 \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Install gazebo and navigation2 packages
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
    ros-humble-gazebo-*\
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Install turtlebot3
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3* \
    && rm -rf /var/lib/apt/lists/*

# Install models
RUN git clone https://github.com/osrf/gazebo_models.git \
    && sudo cp -rf gazebo_models/* /usr/share/gazebo-11/models \
    && rm -rf gazebo_models

ENV GAZEBO_MODEL_PATH $GAZEBO_MODEL_PATH:/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models

# Add user `docker`, set password to `docker` and add it to group `sudo`
RUN useradd -m docker -s /bin/bash && echo "docker:docker" | chpasswd && adduser docker sudo
USER docker

# Add ROS paths to docker user
RUN echo "source /opt/ros/humble/setup.bash" >> /home/docker/.bashrc

WORKDIR /home/docker/dev_ws
