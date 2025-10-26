# file: physical_nav2_launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # --- Launch args ---
    use_sim_time     = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    autostart        = LaunchConfiguration('autostart')
    with_slam        = LaunchConfiguration('with_slam')       # true -> SLAM Toolbox; false -> AMCL+map
    start_nav2       = LaunchConfiguration('start_nav2')
    start_waypoints  = LaunchConfiguration('start_waypoints')
    start_zenoh      = LaunchConfiguration('start_zenoh')     # optional; OFF by default
    start_tools      = LaunchConfiguration('start_tools')     # rqt_graph / rviz / rqt_steering

    declare_use_sim_time     = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([get_package_share_directory('basestation'), 'config', 'slam_params.yaml'])
    )
    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([get_package_share_directory('basestation'), 'config', 'nav2_params.yaml'])
    )
    declare_autostart        = DeclareLaunchArgument('autostart', default_value='true')
    declare_with_slam        = DeclareLaunchArgument('with_slam', default_value='true')
    declare_start_nav2       = DeclareLaunchArgument('start_nav2', default_value='true')
    declare_start_waypoints  = DeclareLaunchArgument('start_waypoints', default_value='true')
    declare_start_zenoh      = DeclareLaunchArgument('start_zenoh', default_value='false')
    declare_start_tools      = DeclareLaunchArgument('start_tools', default_value='true')

    # --- Robot description ---
    pkg_bs = get_package_share_directory('basestation')
    urdf_xacro = os.path.join(pkg_bs, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(urdf_xacro).toxml()

    zenoh_router = Node(
        package='rmw_zenoh_cpp', executable='rmw_zenohd',
        condition=IfCondition(start_zenoh)
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_sim_time)
    )
    joint_state_publisher = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_time)
    )

    rqt_graph = Node(
        package='rqt_graph', executable='rqt_graph', output='screen',
        condition=IfCondition(start_tools)
    )
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(start_tools)
    )
    rqt_steer = Node(
        package='rqt_robot_steering', executable='rqt_robot_steering', arguments=['--force-discover'],
        condition=IfCondition(start_tools)
    )

    # --- Base + EKF ---
    ekf_yaml = os.path.join(pkg_bs, 'config', 'ekf.yaml')
    base_node = Node(
        package='ugv_base_node', executable='base_node_ekf',
        parameters=[{'pub_odom_tf': True}],
        condition=UnlessCondition(use_sim_time)
    )
    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
        parameters=[ekf_yaml],
        remappings=[('/odometry/filtered', '/odom')],
        condition=UnlessCondition(use_sim_time)
    )

    # --- SLAM Toolbox (only if with_slam==true) ---
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={'slam_params_file': slam_params_file, 'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(with_slam)
    )

    # --- Nav2 bringup (bringup_launch.py; pass slam flag) ---
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
            'slam': with_slam
        }.items(),
        condition=IfCondition(start_nav2)
    )

    # --- Waypoint follower ---
    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
        output='screen', parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_waypoints)
    )

    # NOTE: Do NOT add a static map->odom transform; SLAM/AMCL publish it.

    return LaunchDescription([
        declare_use_sim_time, declare_slam_params_file, declare_nav2_params_file,
        declare_autostart, declare_with_slam, declare_start_nav2, declare_start_waypoints,
        declare_start_zenoh, declare_start_tools,
        # zenoh_router,
        robot_state_publisher, joint_state_publisher, joint_state_publisher_gui,
        rqt_graph, rviz, rqt_steer,
        base_node, ekf_node,
        slam_toolbox, nav2, waypoint_follower
    ])
