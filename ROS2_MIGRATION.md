# LiDAR_IMU_Init ROS1 to ROS2 Migration

This document outlines the changes made to migrate the LiDAR_IMU_Init package from ROS1 to ROS2 Jazzy.

## Key Changes Made

### 1. Development Container
- Created `.devcontainer/devcontainer.json` and `.devcontainer/Dockerfile`
- Based on `ros:jazzy` image
- Includes all necessary ROS2 dependencies and development tools

### 2. Package Configuration
- Updated `package.xml` to format 3 with ROS2 dependencies:
  - Changed from `catkin` to `ament_cmake` build system
  - Replaced `roscpp`, `rospy` with `rclcpp`, `rclpy`
  - Updated `tf` to `tf2`, `tf2_ros`, `tf2_geometry_msgs`
  - Changed message dependencies format
  - Added `rosidl_default_generators` and `rosidl_default_runtime`

### 3. CMakeLists.txt
- Updated to use `ament_cmake` instead of `catkin`
- Changed C++ standard from C++14 to C++17
- Updated package finding to use `find_package` for ROS2 packages
- Replaced `add_message_files`/`generate_messages` with `rosidl_generate_interfaces`
- Updated target linking with `ament_target_dependencies`
- Added proper install directives for executables and resources
- Added `ament_package()` at the end

### 4. Source Code Updates

#### Headers and Includes
- Updated `common_lib.h` to use ROS2 message headers:
  - `sensor_msgs/msg/imu.hpp`
  - `nav_msgs/msg/odometry.hpp`
  - `lidar_imu_init/msg/states.hpp`
  - `lidar_imu_init/msg/pose6_d.hpp`
- Replaced `tf/transform_broadcaster.h` with `tf2_ros/transform_broadcaster.h`
- Updated to use `tf2_geometry_msgs` for transformations

#### Main Node (laserMapping.cpp)
- Converted from ROS1 callback-based structure to ROS2 Node class
- Updated message types from ROS1 to ROS2 format:
  - `sensor_msgs::Imu` → `sensor_msgs::msg::Imu`
  - `nav_msgs::Odometry` → `nav_msgs::msg::Odometry`
  - `nav_msgs::Path` → `nav_msgs::msg::Path`
- Replaced `ros::NodeHandle` with `rclcpp::Node`
- Updated publishers/subscribers to ROS2 format
- Changed parameter handling from `nh.param<>()` to `declare_parameter()`/`get_parameter()`
- Updated time handling from `ros::Time` to `rclcpp::Time`
- Replaced `ros::Duration` with `rclcpp::Duration`

#### Preprocessing (preprocess.h)
- Updated includes to use ROS2 headers
- Replaced `sensor_msgs/PointCloud2.h` with `sensor_msgs/msg/point_cloud2.hpp`

### 5. Launch Files
- Created new ROS2 Python launch files in `launch_ros2/` directory
- Updated from XML format to Python launch format
- Added conditional RViz launching
- Updated parameter passing mechanism

### 6. Message Definitions
- Message files in `msg/` directory remain the same format
- Updated CMakeLists.txt to properly generate ROS2 interfaces

## Still Required for Full Migration

### 1. LiDAR Driver Dependencies
- Need to replace `livox_ros_driver` with `livox_ros_driver2`
- Update Livox message handling in preprocess.cpp
- Ensure compatibility with other LiDAR drivers (Velodyne, Ouster, etc.)

### 2. Complete Function Migration
- Update all callback functions to use ROS2 shared_ptr message format
- Replace `ConstPtr` with `SharedPtr` throughout
- Update TF2 transformation logic
- Fix Python plotting integration if needed

### 3. Testing and Validation
- Test with actual LiDAR and IMU hardware
- Validate parameter loading from YAML files
- Ensure all ROS2 message publishing/subscribing works correctly
- Test launch files and RViz visualization

### 4. Documentation Updates
- Update README.md with ROS2 build and usage instructions
- Update configuration file documentation
- Add ROS2-specific troubleshooting information

## Build Instructions

```bash
# In the dev container or ROS2 Jazzy environment:
cd /workspace
colcon build --packages-select lidar_imu_init
source install/setup.bash

# Run with launch file:
ros2 launch lidar_imu_init livox_avia.launch.py
```

## Dependencies to Install

The dev container includes most dependencies, but you may need:
- livox_ros_driver2 (for Livox LiDAR support)
- Additional LiDAR driver packages for ROS2
- PCL 1.8+ with ROS2 bindings

This migration provides a solid foundation for running the LiDAR_IMU_Init package in ROS2 Jazzy while maintaining the core functionality and algorithms.
