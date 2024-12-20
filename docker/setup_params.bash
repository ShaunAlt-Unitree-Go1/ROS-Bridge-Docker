#!/bin/bash
# =============================================================================
# Setup Parameters File
# Created by: Shaun Altmann
# =============================================================================


# =============================================================================
# Get Script Directory
# =============================================================================
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"


# =============================================================================
# Create File
# =============================================================================
echo "topics:" > $SCRIPT_DIR/bridge.yaml


# =============================================================================
# Write Namespace Parameters
# =============================================================================
if [ $# -eq 0 ]; then
echo "
  - topic: /scan
    type: sensor_msgs/msg/LaserScan
    queue_size: 10
  - topic: /cmd_vel
    type: geometry_msgs/msg/Twist
  - topic: /odom
    type: nav_msgs/msg/Odometry
    queue_size: 10" >> $SCRIPT_DIR/bridge.yaml
else
for robot_name in "$@"; do
echo "
  - topic: $robot_name/scan
    type: sensor_msgs/msg/LaserScan
    queue_size: 10
  - topic: $robot_name/cmd_vel
    type: geometry_msgs/msg/Twist
  - topic: $robot_name/odom
    type: nav_msgs/msg/Odometry
    queue_size: 10" >> $SCRIPT_DIR/bridge.yaml
done
fi


# =============================================================================
# Write Generic Parameters
# =============================================================================
echo "
  - topic: /tf
    type: tf2_msgs/msg/TFMessage
  - topic: /tf_static
    type: tf2_msgs/msg/TFMessage" >> $SCRIPT_DIR/bridge.yaml


exit 0
# =============================================================================
# End of File
# =============================================================================
