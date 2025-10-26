import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node


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
                    }] 
    )

    # Run the node
    return LaunchDescription([
        lidar_node
    ])




#  ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node   --ros-args   
# -p product_name:=LDLiDAR_LD06   -p topic_name:=scan   -p frame_id:=base_laser   
# -p port_name:=/dev/ttyACM0   -p port_baudrate:=230400  
# -p laser_scan_dir:=true   -p enable_angle_crop_func:=false