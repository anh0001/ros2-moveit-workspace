# Use official MoveIt2 Docker image as base
FROM moveit/moveit2:humble-release

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install additional tools and dependencies
RUN apt-get update && apt-get install -y \
    # Development tools
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    nano \
    sudo \
    software-properties-common \
    # X11 and GUI tools
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    # ROS 2 development tools
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # MoveIt dependencies
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-setup-assistant \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-moveit-ros-visualization \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-visual-tools \
    # Additional useful packages
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    # Common robot packages
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    # Utilities
    htop \
    tmux \
    tree \
    # Clean up
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set up workspace
RUN mkdir -p /workspace/src && chown -R $USERNAME:$USERNAME /workspace

# Switch to non-root user
USER $USERNAME
WORKDIR /workspace

# Set up ROS 2 environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc \
    && echo "alias cb='cd /workspace && colcon build --symlink-install'" >> ~/.bashrc \
    && echo "alias cbs='cd /workspace && colcon build --symlink-install --packages-select'" >> ~/.bashrc \
    && echo "alias setup='source /workspace/install/setup.bash'" >> ~/.bashrc

# Initialize rosdep (run as root first)
USER root
RUN rosdep update || true

# Switch back to non-root user
USER $USERNAME

# Set the entrypoint
ENTRYPOINT ["/bin/bash"]
