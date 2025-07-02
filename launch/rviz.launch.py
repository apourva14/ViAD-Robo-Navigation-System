import os 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_name = get_package_share_directory('rav_bot')
    pkg_path = os.path.join(pkg_name)

    rviz_config = os.path.join(pkg_name,'rviz','rav_bot.rviz')

    rsp_file = os.path.join(pkg_name,'launch','rsp.launch.py')

    rsp_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rsp_file])
    )

    left_wheel_tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf2_ros_left_wheel',
        output='screen',
        arguments=['0', '0.1485', '0', '0', '0', '-1.570795', 'base_link', 'left_wheel']
    )

    right_wheel_tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf2_ros_right_wheel',
        output='screen',
        arguments=['0', '-0.1485', '0', '0', '0', '-1.570795', 'base_link', 'right_wheel']
        
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
        # arguments=['-d', LaunchConfiguration(rviz_config)]
    )
    
    return LaunchDescription([
        rsp_include,
        left_wheel_tf2_node,
        right_wheel_tf2_node,
        rviz_node
    ])