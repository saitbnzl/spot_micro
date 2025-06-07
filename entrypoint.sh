#!/bin/bash
set -e

# setup ros2 environment
source /ros2_ws/install/setup.bash

# execute whatever command was passed
exec "$@"
