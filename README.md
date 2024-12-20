# ROS-Bridge-Docker
Creates a Docker for bridging ROS1 Noetic and ROS2 Galactic topics.

## Table of Contents
- [Contributors](#contributors)
- [See Also](#see-also)
- [Setup](#setup)
- [Usage](#usage)
    - [Running the Docker](#running-the-docker)
    - [Using with a Physical Robot](#using-with-a-physical-robot)
    - [Using with a Simulation](#using-with-a-simulation)

## Contributors
Created by [Shaun Altmann](https://github.com/ShaunAlt).

## See Also
This bridge runs inside of a Docker that runs Ubuntu 20.04, ROS Noetic, and ROS2 Galactic. This bridge can be used to implement ROS2 Navigation ([Go1-Nav2-SDK](https://github.com/ShaunAlt-Unitree-Go1/Go1-Nav2-SDK)) with a Physical or Simulated ([Unitree-Go1-Sim](https://github.com/ShaunAlt-Unitree-Go1/Unitree-Go1-Sim)) robot. If connected to a physical Go1 robot, you will also need to use the [ROS-Bridge-Noetic](https://github.com/ShaunAlt-Unitree-Go1/ROS-Bridge-Noetic) repository to help connect the required topics.

## Setup
This repository requires the use of Docker on an Ubuntu VM. You can learn how to create an Ubuntu VM [here](https://github.com/ShaunAlt-Unitree-Go1#creating-ubuntu-vms), and install Docker on your VM [here](https://github.com/ShaunAlt-Unitree-Go1#installing-docker).

Once the above has been completed, you can clone this repository and build the docker using the setup scripts.
``` bash
$ cd ~
$ git clone https://github.com/ShaunAlt-Unitree-Go1/ROS-Bridge-Docker.git
$ ./ROS-Bridge-Docker/docker/build_docker.bash
```

## Usage
### Running the Docker
If you do not current have this repository's docker image running, use the following command:
``` bash
$ cd ~
$ ./ROS-Bridge-Docker/docker/run_docker.bash
```

If you do already have this repository's docker image running, use the following command:
``` bash
$ cd ~
$ ./ROS-Bridge-Docker/docker/run_docker.bash -t
```

You can use the `-h` option to show other options for running this docker image.

### Using with a Physical Robot
> [!NOTE]
> This tutorial assumes that you have not namespaced the physical robot, and hence it is the only one you are trying to connect to.

See [here](https://github.com/ShaunAlt-Unitree-Go1#connecting-to-a-physical-unitree-go1-robot) for steps on how to turn on and connect to a physical Unitree Go1 robot.

Once this has happened, you will need to make sure that the docker is able to connect to the robot, and that all of the configuration is correct.
``` bash
# on your machine
$ cd ~/
$ ./ROS-Bridge-Docker/docker/run_docker.bash

# in the docker container
$ cat /etc/hosts # the `192.168.123.161 raspberrypi` should be in there
$ ssh unitree@192.168.123.15 # attempt to ssh into the robot's head from the docker
Password: 123
```

If you are able to connect to the Go1 robot from the Docker, you can open up 3 terminals and use the following commands to bridge the required ROS data into ROS2.
1. Create a Dynamic Bridge.
    ``` bash
    # on your machine
    $ cd ~/
    $ ./ROS-Bridge-Docker/docker/run_docker.bash

    # in the docker container
    $ cd /home/rosuser/bridge_ws/
    $ source install/setup.bash
    $ export ROS_MASTER_URI='https://192.168.123.161:11311'
    $ ros2 run ros1_bridge dynamic_bridge
    ```
2. Create a Lidar Data Relay.
    ``` bash
    # on your machine
    $ cd ~/
    $ ./ROS-Bridge-Docker/docker/run_docker.sh -t

    # in the docker container
    $ cd /home/rosuser/noetic_ws/
    $ source devel/setup.bash
    $ export ROS_MASTER_URI='https://192.168.123.161:11311'
    $ rosrun relay_physical lidar_relay.py
    ```
3. Create a Transform Re-Publisher.
    ``` bash
    # on your machine
    $ cd ~/
    $ ./ROS-Bridge-Docker/docker/run_docker.sh -t

    # in the docker container
    $ cd /home/rosuser/noetic_ws/
    $ source devel/setup.bash
    $ export ROS_MASTER_URI='https://192.168.123.161:11311'
    $ rosrun relay_physical tf_republish.py
    ```

### Using with a Simulation
Before running this bridge, you need to have already started up the [Unitree-Go1-Sim](https://github.com/ShaunAlt-Unitree-Go1/Unitree-Go1-Sim).

> [!NOTE]
> This tutorial makes use of the Parameter Bridge, as it reduces the amount of data being transferred between ROS1 and ROS2. You could use the Dynamic Bridge, however it is not recommended.

When implementing the Parameter Bridge, you will need to use the `setup_params.bash` script which will create the parameters yaml file. If you just want the generic topics, use no arguments when calling it. If your simulated robots have been namespaced, write each of those namespaces as an argument to the script (see below for examples).
- If you are simulating a single robot that is not namespaced, use the following command:
    ``` bash
    $ ./setup_params.bash
    ```
- If you are simulating a single robot that is namespaced (e.g. 'robot1'), use the following command:
    ``` bash
    $ ./setup_params.bash robot1
    ```
- If you are simulating multiple robots (e.g. 'robot_1', 'robot_2', 'robot_3'), use the following command:
    ``` bash
    $ ./setup_params.bash robot_1 robot_2 robot_3
    ```

Open up a new terminal and use the following commands to create the required ROS Bridge.
``` bash
# on your machine
$ cd ~/
$ ./ROS-Bridge-Docker/docker/run_docker.bash

# in the docker container
$ cd /home/rosuser/bridge_ws/
$ source install/setup.bash
$ ./setup_params.bash # see above for details on what arguments to pass
$ rosparam load bridge.yaml
$ ros2 run ros1_bridge parameter_bridge
```
