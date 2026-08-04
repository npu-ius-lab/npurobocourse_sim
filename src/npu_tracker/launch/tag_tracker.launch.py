import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('npu_tracker')
    default_config = os.path.join(pkg_share, 'config', 'tag_tracker.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time from /clock.'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Tag tracker parameters (hover height, limits, timeouts).'),
        Node(
            package='npu_tracker',
            executable='tag_tracker',
            name='tag_tracker',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
    ])
