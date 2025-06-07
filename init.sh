#!/bin/bash

source /opt/ros/humble/setup.sh
cd ~/ros2_ws/
colcon build
source ~/ros2_ws/install/setup.sh
echo 'Welcome to Spot Micro brain!'
