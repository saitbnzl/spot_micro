FROM ros:foxy-ros-base
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
RUN apt-get update && apt-get install -y \
    tmux \
    zsh \
    curl \
    wget \
    vim \
    sudo \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y python3-pip
RUN pip3 install adafruit-circuitpython-servokit Jetson.GPIO

COPY 99-gpio.rules /etc/udev/rules.d/99-gpio.rules

ARG USER=spot
ARG UID=1000
ARG GID=1000
ARG PW=micro

RUN groupadd -g 108 i2c && \
    groupadd -f -r gpio && \
    useradd -m ${USER} --uid=${UID} && \
    echo "${USER}:${PW}" | chpasswd && \
    usermod -a -G gpio ${USER} && \
    usermod -a -G i2c ${USER} && \
    usermod -a -G sudo ${USER}

WORKDIR /home/${USER}

# Create directory and clone your repo
RUN  mkdir -p /ros2_ws/src/ && \
    cd /ros2_ws/src/ && \
    git clone https://github.com/saitbnzl/spot_micro.git

# Make sure the script is executable
RUN chmod +x /ros2_ws/src/spot_micro/entrypoint.sh

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions

RUN . /opt/ros/foxy/setup.sh && \
    cd /ros2_ws && \
    colcon build --symlink-install

ENTRYPOINT ["/ros2_ws/src/spot_micro/entrypoint.sh"]

USER ${UID}:${GID}
CMD ["bash"]
