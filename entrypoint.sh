#!/usr/bin/env bash
set -e

# 1) ROS distro
source /opt/ros/humble/setup.bash

# 2) Your workspace
source /home/spot/ros2_ws/install/setup.bash

# 3) Give control to whatever the user asked
exec "$@"
