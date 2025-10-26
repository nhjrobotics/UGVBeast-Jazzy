import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    autostart = LaunchConfiguration('autostart')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulated time'
    )
    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('basestation'), 'config', 'slam_params.yaml'
        ]),
        description='Path to slam_toolbox params'
    )
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('basestation'), 'config', 'nav2_params.yaml'
        ]),
        description='Path to Nav2 params'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true', description='Autostart lifecycle nodes'
    )

    # RSP
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('basestation'), 'launch', 'rsp_launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': str(os.path.join(get_package_share_directory('basestation'),
                                        'worlds', 'mars_simulation_world.sdf')),
            'on_exit_shutdown': 'true'
        }.items(),
    )

    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'thebeast', '-z', '3.07'],
        output='screen'
    )

    # Bridge (adds tf_static)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
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
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ],
        output='screen'
    )

    # SLAM Toolbox (online async)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Nav2 bringup (slam = True -> no AMCL)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
            'slam': 'True'
        }.items()
    )

    # Waypoint follower (managed by Nav2 lifecycle manager from params file)
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time, declare_slam_params, declare_nav2_params, declare_autostart,
        rsp, gazebo, bridge, spawn_entity, slam, nav2, waypoint_follower
    ])
