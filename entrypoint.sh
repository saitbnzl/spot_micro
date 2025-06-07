#!/bin/bash
set -e

# Setup the ROS 2 Humble environment
source /opt/ros/humble/setup.bash

WORKSPACE_PATH="/home/spot/ros2_ws"

# Build the workspace if it's not already built
if [ ! -f "${WORKSPACE_PATH}/install/setup.bash" ]; then
  echo "--- ROS 2 workspace not found. Building now... ---"
  cd ${WORKSPACE_PATH}
  colcon build
  echo "--- Build complete. ---"
fi

# Source the local workspace
source ${WORKSPACE_PATH}/install/setup.bash

exec "$@"
