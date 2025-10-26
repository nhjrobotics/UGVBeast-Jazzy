import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

import xacro


def generate_launch_description():

    # Configure the node
    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='lidlidar_publisher',
        output='screen',
        parameters=[{'product_name': 'LDLiDAR_LD06',
                     'topic_name': 'scan',
                     'frame_id': 'base_lidar_link',
                     'port_name': '/dev/ttyACM0',
                     'port_baudrate':230400,
                     'laser_scan_dir':True,
                     'enable_angle_crop_func':False
                    }],
    )

    ugv_bringup = Node(
        package='ugv_bringup',
        executable='ugv_bringup',
        output='screen',
    )

    ugv_driver = Node(
        package='ugv_bringup',
        executable='ugv_driver',
        output='screen',
    )

    # Launch!
    return LaunchDescription([
  
        lidar_node,
        ugv_bringup,
        ugv_driver,
    ])
