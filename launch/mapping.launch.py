#!/usr/bin/env python3
import os
import rclpy

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Allow toggling sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Find SLAM Toolbox’s own launch file
    slam_pkg_share = get_package_share_directory('slam_toolbox')
    slam_launch = os.path.join(
        slam_pkg_share, 'launch', 'online_async_launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        ),
    ])
