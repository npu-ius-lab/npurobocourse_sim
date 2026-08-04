from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time from /clock.'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='npu_robot_lidar_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.055',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'npu_robot/base_link',
                '--child-frame-id', 'npu_robot/lidar_link',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
