import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.substitutions import FindExecutable


from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():

    # Launch RSP
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('basestation'), 'launch', 'rsp_launch.py')]),
        launch_arguments={'use_sim_time': ['true']}.items(),
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': [str(os.path.join(get_package_share_directory('basestation'), 'worlds', 'mars_simulation_world.sdf'))], 
        'on_exit_shutdown': 'true'}.items(),
    )

    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                  '-name', 'thebeast',
                  '-z', '3.07'],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/imu_sensor@sensor_msgs/msg/Imu@gz.msgs.IMU',
                   '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/turret_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/turret_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                   '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                   '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        bridge,
        spawn_entity,
    ])
