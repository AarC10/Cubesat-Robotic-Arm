FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm-256color \
    ROS_DISTRO=humble \
    COLCON_WS=/workspace

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    cmake \
    ninja-build \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-argcomplete \
    locales \
    gosu \
    sudo \
    procps \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

# Workspace skeleton
RUN mkdir -p ${COLCON_WS}/src
WORKDIR ${COLCON_WS}

RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /etc/bash.bashrc && \
    echo 'if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi' >> /etc/bash.bashrc

COPY docker_entrypoint.sh /ros_entry.sh
RUN chmod +x /ros_entry.sh

ENTRYPOINT ["/ros_entry.sh"]
CMD ["bash"]

