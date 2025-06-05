#!/bin/bash
set -e

# setup ros2 environment
source /ros2_ws/install/setup.bash

# drop to user shell
exec su -s /bin/bash $USER
