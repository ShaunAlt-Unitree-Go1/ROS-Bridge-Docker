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
# Setup ROS Keys
# =============================================================================
RUN apt-get install curl -y

# ===============
# ROS Noetic Keys
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# =================
# ROS Galactic Keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null


# =============================================================================
# Update Debian Packages
# =============================================================================
RUN apt-get update -y
RUN apt-get upgrade -y


# =============================================================================
# Install Debian Packages
# =============================================================================
RUN apt-get install -y \
    ros-noetic-ros-base \
    ros-galactic-ros-base \
    git


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
RUN rosdep update --rosdistro noetic
RUN rosdep update --rosdistro galactic


# =============================================================================
# Clone + Build Simulation from GitHub Repo
# =============================================================================
WORKDIR /home/rosuser/bridge_ws/src
RUN git clone https://github.com/ros2/ros1_bridge.git
WORKDIR /home/rosuser/bridge_ws
RUN rosdep install --from-paths src --ignore-src --rosdistro noetic -y
RUN rosdep install --from-paths src --ignore-src --rosdistro galactic -y
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && source /opt/ros/galactic/setup.bash && colcon build --packages-select ros1_bridge --cmake-force-configure"


# =============================================================================
# Set Default User
# =============================================================================
USER rosuser


# =============================================================================
# End of File
# =============================================================================
