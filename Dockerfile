# ─── Dockerfile ───────────────────────────────────────────────────────────
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ENV LANG=en_US.UTF-8
ARG USER=spot
ARG UID=1000
ARG GID=1000
ARG GPIO_GID=997
ARG I2C_GID=998
ARG PW=micro
ARG REPO_URL="https://github.com/saitbnzl/spot_micro"

# 1) System + GPIO/I2C bits
RUN apt-get update && apt-get install -y \
     locales python3-pip python3-lgpio liblgpio-dev \
     git tmux zsh curl wget vim sudo \
  && locale-gen en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/* \
  && pip3 install --no-cache-dir adafruit-circuitpython-servokit adafruit-blinka

COPY 99-gpio.rules /etc/udev/rules.d/99-gpio.rules

# 2) Create your “spot” group with GID 1000, then gpio/i2c, then useradd --gid=1000
RUN groupadd -f -r -g ${GPIO_GID} gpio \
  && groupadd -f -r -g ${I2C_GID} i2c \
  && useradd -m ${USER} --uid=${UID} \
  && echo "${USER}:${PW}" | chpasswd \
  && usermod -aG gpio,i2c,sudo ${USER}

WORKDIR /home/${USER}

# 3) Clone, build, entrypoint...
RUN mkdir -p ros2_ws/src \
 && git clone --depth=1 ${REPO_URL} ros2_ws/src/spot_micro \
 && cd ros2_ws \
 && . /opt/ros/humble/setup.bash \
 && colcon build --symlink-install

COPY entrypoint.sh /home/${USER}/entrypoint.sh
RUN chmod +x /home/${USER}/entrypoint.sh


USER spot


ENTRYPOINT ["/home/spot/entrypoint.sh"]
CMD ["bash"]
