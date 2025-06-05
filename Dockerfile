FROM ros:rolling-ros-base

# Set UTF-8 locale
RUN apt-get update && apt-get install -y locales && locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# Install system packages
RUN apt-get update && apt-get install -y \
    tmux \
    zsh \
    curl \
    wget \
    vim \
    sudo \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create and activate Python virtual environment
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"
ENV PYTHONPATH="/opt/venv/lib/python3/site-packages:$PYTHONPATH"

# Install Python packages in venv
RUN /opt/venv/bin/pip install --upgrade pip \
    && /opt/venv/bin/pip install \
        adafruit-circuitpython-servokit \
        Jetson.GPIO \
        empy \
        catkin_pkg \
	numpy

# Setup ROS workspace
ARG USER=spot
ARG UID=1001
ARG GID=1001
ARG PW=8905

# Create user and groups
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

WORKDIR /home/${USER}

# Create directory and clone your repo
RUN  mkdir -p /ros2_ws/src/ && \
    cd /ros2_ws/src/ && \
    git clone https://github.com/saitbnzl/spot_micro.git



# Make sure the script is executable
RUN chmod +x /ros2_ws/src/spot_micro/entrypoint.sh
RUN chmod +x /ros2_ws/src/spot_micro/docker_install.sh

RUN bash /ros2_ws/src/spot_micro/docker_install.sh


RUN . /opt/ros/rolling/setup.sh && \
    cd /ros2_ws && \
    colcon build --symlink-install

ENTRYPOINT ["/ros2_ws/src/spot_micro/entrypoint.sh"]
RUN chmod +x /ros2_ws/src/spot_micro/entrypoint.sh

CMD ["bash"]
