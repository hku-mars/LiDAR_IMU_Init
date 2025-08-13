#!/bin/bash

# ROS2 Build Script for LiDAR_IMU_Init

set -e

echo "=== ROS2 LiDAR_IMU_Init Build Script ==="

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Create workspace if it doesn't exist
if [ ! -d "/workspace/src" ]; then
    echo "Creating ROS2 workspace structure..."
    mkdir -p /workspace/src
    cd /workspace/src
    ln -sf /workspace /workspace/src/lidar_imu_init
fi

cd /workspace

# Update rosdep
echo "Updating rosdep..."
rosdep update || echo "rosdep update failed, continuing..."

# Install dependencies
echo "Installing dependencies..."
rosdep install --from-paths src --ignore-src -r -y || echo "Some dependencies may not be available, continuing..."

# Build the package
echo "Building the package..."
colcon build --packages-select lidar_imu_init --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

echo "=== Build completed successfully! ==="
echo ""
echo "To run the package:"
echo "  source install/setup.bash"
echo "  ros2 launch lidar_imu_init livox_avia.launch.py"
echo ""
echo "To run individual nodes:"
echo "  ros2 run lidar_imu_init li_init"
