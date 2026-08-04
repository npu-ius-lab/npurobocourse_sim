import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('npu_tracker')
    default_tasks = os.path.join(pkg_share, 'config', 'tasks.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    tasks_file = LaunchConfiguration('tasks_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time from /clock.'),
        DeclareLaunchArgument(
            'tasks_file', default_value=default_tasks,
            description='Mission parameters (takeoff, ring, waypoints, limits).'),
        Node(
            package='npu_tracker',
            executable='mission',
            name='mission',
            output='screen',
            parameters=[tasks_file, {'use_sim_time': use_sim_time}],
        ),
    ])
