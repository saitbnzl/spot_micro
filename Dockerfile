# Use the modern, LTS ROS 2 Humble base image (based on Ubuntu 22.04)
FROM ros:humble-ros-base

ENV LANG=en_US.UTF-8

# Install dependencies, including the lgpio packages which are available on Ubuntu 22.04
RUN apt-get update && apt-get install -y \
    locales \
    python3-pip \
    git \
    tmux \
    zsh \
    curl \
    wget \
    vim \
    sudo \
    liblgpio-dev \
    python3-lgpio \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install adafruit-circuitpython-servokit adafruit-blinka

COPY 99-gpio.rules /etc/udev/rules.d/99-gpio.rules

ARG USER=spot
ARG UID=1000
ARG GID=1000
ARG PW=micro

RUN useradd -m ${USER} --uid=${UID} && echo "${USER}:${PW}" | \
    chpasswd && \
    groupadd -f -r gpio && \
    groupadd --gid 108 i2c && \
    usermod -a -G gpio ${USER} && \
    usermod -a -G i2c ${USER} && \
    usermod -a -G sudo ${USER}

USER ${UID}:${GID}
WORKDIR /home/${USER}

# The path to the entrypoint is now absolute for robustness
ENTRYPOINT ["/home/spot/ros2_ws/src/spot_micro/entrypoint.sh"]
CMD ["bash"]
