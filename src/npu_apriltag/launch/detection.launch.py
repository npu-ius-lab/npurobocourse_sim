import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    camera_preset = LaunchConfiguration('camera_preset').perform(context)

    if camera_preset not in ('forward', 'down'):
        raise RuntimeError(
            f"unknown camera_preset '{camera_preset}', expected 'forward' or 'down'")

    description_share = get_package_share_directory('npu_sim_description')
    apriltag_share = get_package_share_directory('npu_apriltag')
    preset_path = os.path.join(
        description_share, 'config', 'camera_presets', camera_preset + '.yaml')
    with open(preset_path) as f:
        preset = yaml.safe_load(f)
    config_path = os.path.join(apriltag_share, 'config', 'detector.yaml')

    detector = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_detector',
        namespace='npu/drone',
        output='screen',
        parameters=[config_path, {'use_sim_time': use_sim_time}],
        remappings=[
            ('image_rect', '/npu/drone/camera/image_raw'),
            ('camera_info', '/npu/drone/camera/camera_info'),
        ],
    )

    adapter = Node(
        package='npu_apriltag',
        executable='detection_adapter',
        name='detection_adapter',
        output='screen',
        parameters=[config_path, {
            'use_sim_time': use_sim_time,
            'camera_translation': [float(v) for v in preset['translation']],
            'camera_rpy': [float(v) for v in preset['rpy']],
        }],
    )

    return [detector, adapter]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time from /clock.'),
        DeclareLaunchArgument(
            'camera_preset', default_value='forward',
            description="Camera mount preset: 'forward' or 'down'."),
        OpaqueFunction(function=_setup),
    ])
