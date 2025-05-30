#!/bin/bash
set -e

# Locale setup
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources and keys
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Foxy base and python tools
sudo apt update
sudo apt install -y ros-foxy-ros-base python3-rosdep python3-colcon-common-extensions python3-pip

# Initialize rosdep
sudo rosdep init || true
rosdep update

echo "ROS 2 Foxy base installed. To use ROS 2, source the setup script:"
echo "  source /opt/ros/foxy/setup.bash"
