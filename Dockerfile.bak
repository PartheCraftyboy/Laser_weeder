FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=dev
ARG HOST_UID=1000
ARG HOST_GID=1000

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
      ros-jazzy-ros-gz \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-rqt-common-plugins \
      git vim less curl sudo \
    && rm -rf /var/lib/apt/lists/*

RUN userdel -r ubuntu 2>/dev/null || true \
    && groupadd -g ${HOST_GID} ${USERNAME} 2>/dev/null || true \
    && useradd -m -u ${HOST_UID} -g ${HOST_GID} -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "[ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
