# ROS-Bridge-Docker
Creates a Docker for bridging ROS1 Noetic and ROS2 Galactic topics.

## Table of Contents
- [Dependencies](#dependencies)
- [Setup](#setup)
- [Running the Docker](#running-the-docker)
- [Running the Bridge](#running-the-bridge)

## Dependencies
This project relies on the [ros2/ros1_bridge](https://github.com/ros2/ros1_bridge/tree/galactic) repository to create the ROS bridge.

## Setup
1. Clone this repository into a workspace.
    ``` bash
    $ git clone https://github.com/ShaunAlt-Unitree-Go1/ROS-Bridge-Docker.git ~/ros_bridge
    $ cd ~/ros_bridge
    ```
2. Use the `build_docker.bash` script to build the docker image.
    ``` bash
    $ ./build_docker.bash
    ```

## Running the Docker
1. If the docker is not currently running, use the following command:
    ``` bash
    $ ./run_docker.bash
    ```
2. If the docker is currently running, but you want to open a new terminal, use the following command:
    ``` bash
    $ ./run_docker.bash -t
    ```

## Running the Bridge
- This bridge requires that `roscore` has already been run (i.e. you have already launched a ROS1 package). If this has not happened, you will need to open a new docker terminal, and run the following commands to run the roscore.
    ``` bash
    $ source /opt/ros/noetic/setup.bash
    $ roscore
    ```
- Source the ROS bridge before using any of the other commands.
    ``` bash
    $ cd /home/rosuser/bridge_ws # should already be here
    $ source install/setup.bash
    ```
- If you just want to bridge topics, use the `dynamic_bridge`. This will allow you to dynamically bridge topics (use the `--help` option to get more information).
    ``` bash
    $ ros2 run ros1_bridge dynamic_bridge
    ```