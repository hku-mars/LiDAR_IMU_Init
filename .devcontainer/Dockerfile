FROM ros:jazzy

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    curl \
    nano \
    vim \
    gdb \
    libeigen3-dev \
    libpcl-dev \
    libceres-dev \
    python3-matplotlib \
    python3-numpy \
    libomp-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 additional packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-eigen3-cmake-module \
    ros-jazzy-rclcpp \
    ros-jazzy-rclpy \
    ros-jazzy-launch-ros \
    ros-jazzy-launch-xml \
    ros-jazzy-ament-cmake \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-default-runtime \
    ros-jazzy-builtin-interfaces \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace
RUN mkdir -p /workspace
WORKDIR /workspace

# Source ROS 2 setup in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Initialize rosdep
RUN rosdep init || echo "rosdep already initialized"

CMD ["/bin/bash"]
