# LiDAR_IMU_Init - ROS2 Jazzy Version

A robust LiDAR-IMU initialization and calibration system migrated from ROS1 to ROS2 Jazzy.

## Features
- Real-time LiDAR-IMU sensor fusion
- Automatic extrinsic calibration
- Support for multiple LiDAR types (Livox, Velodyne, Ouster, etc.)
- Robust initialization algorithms
- Point cloud processing and mapping

## Prerequisites
- ROS2 Jazzy
- PCL >= 1.8
- Eigen3
- Ceres Solver
- Python 3 (for launch files)

## Quick Start with Dev Container

1. **Open in Dev Container**: The repository includes a complete dev container setup with ROS2 Jazzy.
   ```bash
   # VS Code will prompt to open in dev container
   # Or manually: Ctrl+Shift+P -> "Dev Containers: Reopen in Container"
   ```

2. **Build the package**:
   ```bash
   cd /workspace
   ./build_ros2.sh
   ```

3. **Run with launch file**:
   ```bash
   source install/setup.bash
   ros2 launch lidar_imu_init livox_avia.launch.py
   ```

## Manual Installation (Without Dev Container)

1. **Install ROS2 Jazzy**: Follow [official installation guide](https://docs.ros.org/en/jazzy/Installation.html)

2. **Install dependencies**:
   ```bash
   sudo apt update
   sudo apt install -y \
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
     libeigen3-dev \
     libpcl-dev \
     libceres-dev
   ```

3. **Clone and build**:
   ```bash
   cd ~/ros2_ws/src
   git clone <this-repository>
   cd ~/ros2_ws
   colcon build --packages-select lidar_imu_init
   ```

## Configuration

Configuration files are located in the `config/` directory:
- `avia.yaml` - Livox AVIA configuration
- `horizon.yaml` - Livox Horizon configuration
- `mid360.yaml` - Livox Mid-360 configuration
- `velodyne.yaml` - Velodyne configuration
- `ouster.yaml` - Ouster configuration

## Topics

### Subscribed Topics
- `/livox/lidar` (sensor_msgs/msg/PointCloud2) - LiDAR point cloud
- `/livox/imu` (sensor_msgs/msg/Imu) - IMU measurements

### Published Topics
- `/cloud_registered` (sensor_msgs/msg/PointCloud2) - Registered point cloud
- `/cloud_registered_body` (sensor_msgs/msg/PointCloud2) - Body frame point cloud
- `/aft_mapped_to_init` (nav_msgs/msg/Odometry) - Odometry output
- `/path` (nav_msgs/msg/Path) - Trajectory path

## Parameters

Key parameters (configurable in YAML files):
- `max_iteration`: Maximum iterations for optimization
- `point_filter_num`: Point cloud downsampling factor
- `mapping.filter_size_surf`: Surface filtering size
- `cube_side_length`: Map cube side length
- `initialization.cut_frame`: Enable frame cutting for initialization

## Launch Files

Available launch files in `launch_ros2/`:
- `livox_avia.launch.py` - Livox AVIA setup
- `livox_horizon.launch.py` - Livox Horizon setup
- Additional LiDAR configurations...

## Migration Notes

This package has been migrated from ROS1 to ROS2 Jazzy. Key changes include:
- Updated message types and namespaces
- Converted to rclcpp Node architecture
- Updated TF to TF2
- Python-based launch files
- ROS2 parameter handling

See `ROS2_MIGRATION.md` for detailed migration information.

## Troubleshooting

### Common Issues
1. **Build errors**: Ensure all dependencies are installed
2. **Missing LiDAR drivers**: Install appropriate ROS2 LiDAR drivers
3. **TF2 errors**: Check frame_id naming conventions
4. **Parameter loading**: Verify YAML file paths and format

### Support
- Check the original repository for algorithm details
- ROS2 documentation: https://docs.ros.org/en/jazzy/
- Open issues for ROS2-specific problems

## License
Follows the original BSD license from the source repository.
