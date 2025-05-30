#!/bin/bash
set -e

# setup ros2 environment
source /ros2_ws/install/setup.bash
# create a new shell
exec su -pl $USER --shell=/bin/bash
