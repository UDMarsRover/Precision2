#!/bin/bash
set -e

# Controller device file path. 
# You may need to change this depending on your controller.
# Common device names are 'js0', 'event0', 'event1', etc.
CONTROLLER_DEVICE="/dev/input/js0" 

# Wait for the controller to be connected
echo "Waiting for controller to be connected..."
while [ ! -e "$CONTROLLER_DEVICE" ]; do
  sleep 1
done
echo "Controller detected! Starting ROS 2 launch file."

source /opt/ros/humble/setup.bash
source /home/udmrt/Precision2/install/setup.bash
ros2 launch rover rover_bt.launch.py