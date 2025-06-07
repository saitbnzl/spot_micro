FROM ros:rolling-ros-base

# Set UTF-8 locale
RUN apt-get update && apt-get install -y locales && locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# Install system packages and Python dependencies
RUN apt-get update && apt-get install -y \
    tmux \
    zsh \
    curl \
    wget \
    vim \
    sudo \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages globally
RUN pip3 install --upgrade pip \
    && pip3 install \
        adafruit-circuitpython-servokit \
        Jetson.GPIO \
        empy \
        catkin_pkg \
        numpy \
        lark

# Setup ROS workspace user
ARG USER=spot
ARG UID=1001
ARG GID=1001
ARG PW=8905

RUN groupadd -g ${GID} ${USER} && \
    groupadd -g 108 i2c && \
    groupadd -f -r gpio && \
    useradd -m ${USER} --uid=${UID} --gid=${GID} && \
    echo "${USER}:${PW}" | chpasswd && \
    usermod -a -G gpio ${USER} && \
    usermod -a -G i2c ${USER} && \
    usermod -a -G sudo ${USER}

# Copy udev rules
COPY 99-gpio.rules /etc/udev/rules.d/99-gpio.rules

# Clone project and build
USER ${USER}
WORKDIR /home/${USER}

RUN mkdir -p /ros2_ws/src && \
    cd /ros2_ws/src && \
    git clone https://github.com/saitbnzl/spot_micro.git

# Make entrypoint and install scripts executable
RUN chmod +x /ros2_ws/src/spot_micro/entrypoint.sh \
    && chmod +x /ros2_ws/src/spot_micro/docker_install.sh

# Build ROS2 workspace using system Python
RUN . /opt/ros/rolling/setup.sh && \
    cd /ros2_ws && \
    colcon build --symlink-install

# Use custom entrypoint
ENTRYPOINT ["/ros2_ws/src/spot_micro/entrypoint.sh"]
CMD ["bash"]
