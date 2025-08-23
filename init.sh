#!/bin/bash

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Go to your workspace
cd ~/ros2_ws/

# Build only if you want (optional):
# colcon build

# Source your workspace setup
source ~/ros2_ws/install/setup.bash

echo 'Welcome to Spot Micro brain!'
