# Base image: ROS 2 Humble Desktop Full (per MoveIt setup report)
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Core tools + X11/Qt reliability packages + MoveIt
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    curl \
    git \
    nano \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    sudo \
    vim \
    wget \
    xauth \
    x11-apps \
    x11-utils \
    locales \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libglx-mesa0 \
    mesa-utils \
    qtwayland5 \
    libx11-xcb1 \
    libxkbcommon-x11-0 \
    libxcb-cursor0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shm0 \
    libxcb-xinerama0 \
    ros-humble-moveit \
    ros-humble-moveit-setup-assistant \
    ros-humble-moveit-resources-panda-description \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# UTF-8 locale is required for stable MoveIt Setup Assistant behavior.
RUN locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# Environment defaults for RViz2/MoveIt GUI in containers
ENV QT_X11_NO_MITSHM=1
ENV QT_QPA_PLATFORM=xcb
ENV XDG_RUNTIME_DIR=/tmp/runtime-ros
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Create a non-root user matching typical host UID/GID
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Workspace directory used by devcontainer.json
RUN mkdir -p /workspace/src && chown -R ${USERNAME}:${USERNAME} /workspace
RUN mkdir -p /tmp/runtime-ros && chown -R ${USERNAME}:${USERNAME} /tmp/runtime-ros && chmod 700 /tmp/runtime-ros

USER ${USERNAME}
WORKDIR /workspace

# Shell setup for ROS + workspace overlays
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc \
    && echo "alias glx-check='echo DISPLAY=\$DISPLAY; glxinfo -B'" >> /home/${USERNAME}/.bashrc \
    && echo "alias cb='cd /workspace && colcon build --symlink-install'" >> /home/${USERNAME}/.bashrc \
    && echo "alias cbs='cd /workspace && colcon build --symlink-install --packages-select'" >> /home/${USERNAME}/.bashrc

CMD ["/bin/bash"]
