# =============================================================================
# Unitree Go1 ROS Bridge Dockerfile
# Created by: Shaun Altmann
# 
# Used to create a docker image capable of bridging ROS1 Noetic and ROS2
# Galactic topics.
# =============================================================================

# =============================================================================
# ROS Noetic and Ubuntu 20.04 Base
# =============================================================================
FROM ubuntu:focal-20241011


# =============================================================================
# Prevent User Interaction
# =============================================================================
ENV DEBIAN_FRONTEND=noninteractive


# =============================================================================
# Update Debian Packages
# =============================================================================
RUN apt-get update -y
RUN apt-get upgrade -y


# =============================================================================
# Install Debian Packages
# =============================================================================
RUN apt-get install -y \
    tzdata \
    dirmngr \
    gnupg2 \
    curl \
    lsb-release \
    git \
    build-essential


# =============================================================================
# Setup ROS Keys
# =============================================================================

# ===============
# ROS Noetic Keys
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN bash -c 'curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -'

# =================
# ROS Galactic Keys
RUN bash -c 'curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg'
RUN bash -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null'


# =============================================================================
# Update Debian Packages
# =============================================================================
RUN apt-get update -y
RUN apt-get upgrade -y


# =============================================================================
# Install ROS Packages
# =============================================================================
RUN apt-get install -y \
    ros-noetic-ros-base \
    ros-noetic-sensor-msgs \
    ros-noetic-tf2-tools \
    ros-galactic-ros-base \
    ros-galactic-sensor-msgs \
    ros-galactic-tf2-tools \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-catkin-tools


# =============================================================================
# Add User (rosuser)
# =============================================================================
# Run `useradd --help` for more information on adding a user
RUN useradd \
    -rm \
    -d /home/ubuntu \
    -s /bin/bash \
    -g root \
    -G sudo \
    -u 1000 rosuser \
    -p $(openssl passwd -6 rosuser)
WORKDIR /home/rosuser


# =============================================================================
# Updating ROS Dependencies
# =============================================================================
RUN rosdep init
RUN rosdep update --rosdistro noetic
RUN rosdep update --rosdistro galactic


# =============================================================================
# Clone + Build Noetic Support Packages
# =============================================================================
WORKDIR /home/rosuser/noetic_ws/src/
RUN git clone https://github.com/ShaunAlt-Unitree-Go1/ROS-Bridge-Noetic.git
WORKDIR /home/rosuser/noetic_ws/
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"


# =============================================================================
# Clone + Build ROS Bridge
# =============================================================================
WORKDIR /home/rosuser/bridge_ws/src/
RUN git clone https://github.com/ros2/ros1_bridge.git -b galactic
WORKDIR /home/rosuser/bridge_ws/
COPY params/ /home/rosuser/bridge_ws/params/
COPY setup_params.bash /home/rosuser/bridge_ws/setup_params.bash
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && source /opt/ros/galactic/setup.bash && colcon build --packages-select ros1_bridge --cmake-force-configure"


# =============================================================================
# Set Default User
# =============================================================================
WORKDIR /home/rosuser/
USER rosuser


# =============================================================================
# End of File
# =============================================================================
