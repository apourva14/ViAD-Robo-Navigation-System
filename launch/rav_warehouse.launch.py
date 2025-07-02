#!/usr/bin/python3
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription


def generate_launch_description():

    pkg_name = get_package_share_directory('rav_bot')
    pkg_path = os.path.join(pkg_name)

    # Path to the world file
    world_file = os.path.join(pkg_path, 'world', 'small_warehouse.sdf')

    rsp_file = os.path.join(pkg_path,'launch', 'rsp.launch.py')

    rsp_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rsp_file])
    )

    base_footprint_tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf2_ros_chassie',
        output='screen',
        arguments=['0', '0', '0.033', '0', '0', '0', 'base_footprint', 'base_link']
    )

    # Adjust gazebo server to load the specified world
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spwan_entity',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'my_robot',
            '-z', '0.28',
            '-x', '0',
            '-y', '0',
            '-Y', '0'
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'
        ),
        rsp_include,
        base_footprint_tf2_node,
        gazebo_server,
        gazebo_client,
        spawn_robot
    ])
