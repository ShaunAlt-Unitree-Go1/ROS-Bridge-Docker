#!/bin/bash
# =============================================================================
# Unitree Go1 ROS Bridge - Run Parameter Bridge
# Created by: Shaun Altmann
#
# Used to run the parameter bridge for the Unitree Go1 simulation.
# =============================================================================


# =============================================================================
# Define Constants
# =============================================================================

# =============================
# Location of Bridge Setup File
BRIDGE_FILE="/home/rosuser/bridge_ws/bridge.yaml"

# ======================
# Namespace of the Robot
ROBOT_NAMESPACE="robot1"


# =============================================================================
# Create ROS Bridge Parameter File
# =============================================================================
create_params() {
    # wipe the bridge file
    rm -rf $BRIDGE_FILE
    touch $BRIDGE_FILE

    # create the bridge file data
cat <<EOF >$BRIDGE_FILE
topics:
  -
    topic: /$ROBOT_NAMESPACE/cmd_vel
    type: geometry_msgs/Twist
    queue_size: 1
EOF
}


# =============================================================================
# Run ROS Parameter Bridge
# =============================================================================
run_bridge() {
    source /opt/ros/noetic/setup.bash
    rosparam load $BRIDGE_FILE
    source /home/rosuser/bridge_ws/install/setup.bash
    ros2 run ros1_bridge parameter_bridge
}


# =============================================================================
# Print Script Usage
# =============================================================================
usage() {
    echo "Usage: $0 [-h] [-n name]"
    echo ""
    echo "Purpose:"
    echo "    This script runs the ROS Bridge, bridging the required topics"
    echo "    and services for the Unitree Go1 Simulation."
    echo ""
    echo "Options:"
    echo "    -h        Print this help message."
    echo "    -n name   Specify the robot namespace (default: robot1)."
}


# =============================================================================
# Handle Arguments
# =============================================================================
while getopts "hn:" opt; do
    case "$opt" in
        h)
            usage
            exit 0
            ;;
        n)
            ROBOT_NAMESPACE="$OPTARG"
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            exit 1
            ;;
    esac
done
create_params
run_bridge
exit 0


# =============================================================================
# End of File
# =============================================================================
