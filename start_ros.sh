#!/bin/bash

# Optional: set UID/GID for future use
export UID=$(id -u)
export GID=$(id -g)

docker run -it --rm \
  --net=host \
  --privileged \
  --device /dev/i2c-1 \
  -e USER=$USER \
  -v $HOME/ros2_ws:/home/spot/ros2_ws \
  --name ros2-humble \
  spot:latest
