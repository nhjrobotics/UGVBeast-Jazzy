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

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('basestation'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)


    zenoh_router = Node(
        package='rmw_zenoh_cpp',
        executable='rmw_zenohd',
    )
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )

    node_rqt_graph = Node(
        package='rqt_graph',
        executable='rqt_graph',
        output='screen'
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        #arguments = ["-d " + str(os.path.join(get_package_share_directory("basestation"), 'config', 'rviz_config.rviz'))],
        output='screen'
    )

    rqt_robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        arguments = ["--force-discover"],
        output='screen'
    )

    base_node = Node(
        package='ugv_base_node',
        executable='base_node_ekf',
        parameters=[{'pub_odom_tf': True}],
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("basestation"), 'config', 'ekf.yaml')],
        remappings=[('/odometry/filtered', '/odom')],
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))

    )

    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')]),
        launch_arguments = {'params_file': os.path.join(get_package_share_directory("basestation"), 'config', 'nav2-params.yaml')}.items(),
    )

    # Launch!
    launch_dir = PathJoinSubstitution([FindPackageShare('basestation'), 'launch'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',),

        # zenoh_router,
        node_robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        node_rqt_graph,
        node_rviz,
        rqt_robot_steering,
        base_node,
        ekf_node,
        static_transform,
        #nav2,
    ])
