# spot_micro
This is a collection of ROS 2 nodes for my spot micro. I run ROS 2 on the Jetson Nano in a docker container so this repo also contains a Dockerfile to run the code.

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
