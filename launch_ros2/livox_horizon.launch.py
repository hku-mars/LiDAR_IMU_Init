import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    # Get package directory
    pkg_dir = get_package_share_directory('lidar_imu_init')

    # LiDAR IMU Init Node
    lidar_imu_init_node = Node(
        package='lidar_imu_init',
        executable='li_init',
        name='lidar_imu_init_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'horizon.yaml'),
            {
                'point_filter_num': 2,
                'max_iteration': 5,
                'cube_side_length': 2000.0
            }
        ]
    )

    # RViz Node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_dir, 'rviz_cfg', 'fast_lo.rviz')],
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        lidar_imu_init_node,
        rviz_node
    ])
