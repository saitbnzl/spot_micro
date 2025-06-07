# spot_micro
This is a collection of ROS 2 nodes for my spot micro.

## Build

Build the image with your host's GPIO and I2C group IDs so that the
container can access those devices:

```bash
docker build \
  --build-arg GPIO_GID=$(getent group gpio | cut -d: -f3) \
  --build-arg I2C_GID=$(getent group i2c | cut -d: -f3) \
  -t spot_micro .
```

Supplying these IDs prevents "can not open gpiochip" errors when running
the ROS nodes.
=======
## Building the Docker image

Build the image with the same UID and GID as your host user so that files created inside the container are owned by you. Pass the repository URL so the container can clone the sources:

```bash
docker build --build-arg UID=$(id -u) \
             --build-arg GID=$(id -g) \
             --build-arg REPO_URL=<repo-url> \
             -t spot-gpio:humble .
```

## Running the container

The nodes need access to the GPIO and I2C devices on the Jetson. You can either run the container fully privileged or allow access to just the required devices.

```bash
# give the container full device access
docker run -it --net=host --privileged spot-gpio:humble

# or expose only the GPIO and I2C character devices
docker run -it --net=host \
  --device /dev/gpiochip0 \
  --device /dev/i2c-1 \
  spot-gpio:humble
```

## entrypoint.sh and starting ROS

The image uses `entrypoint.sh` as its entrypoint. The script sources ROS and the workspace then executes whatever command you pass to `docker run`.

For example, to launch the WebSocket bridge directly from the container:

```bash
docker run --net=host --privileged spot-gpio:humble \
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Any other ROS 2 node or launch file can be started in the same way.