import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.actions import Shutdown as ShutdownAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DRONE_SPAWN = {'name': 'npu_drone', 'x': '-1.5', 'y': '-4.0', 'z': '0.32'}
ROBOT_SPAWN = {'name': 'npu_robot', 'x': '2.5', 'y': '-1.5', 'z': '0.03'}
WORLD_NAME = 'course_world'
CLOCK_TIMEOUT_SEC = '60'


def _render_drone_model(preset_name):
    share = get_package_share_directory('npu_sim_description')
    preset_path = os.path.join(share, 'config', 'camera_presets', preset_name + '.yaml')
    template_path = os.path.join(share, 'models', 'drone', 'model.sdf.template')
    with open(preset_path) as f:
        preset = yaml.safe_load(f)
    with open(template_path) as f:
        template = f.read()
    xyz = ' '.join(f'{v:.10g}' for v in preset['translation'])
    rpy = ' '.join(f'{v:.10g}' for v in preset['rpy'])
    rendered = template.replace('{{ camera_xyz }}', xyz).replace('{{ camera_rpy }}', rpy)
    if '{{' in rendered:
        raise RuntimeError('unresolved template token in drone model')
    out = os.path.join(tempfile.mkdtemp(prefix='npu_drone_model_'), 'npu_drone.sdf')
    with open(out, 'w') as f:
        f.write(rendered)
    return out


def _spawn_action(model_file, spec):
    return ExecuteProcess(
        cmd=[
            'bash', '-c',
            'for i in $(seq 1 60); do '
            'ros2 run ros_gz_sim create '
            f'-world {WORLD_NAME} -file {model_file} -name {spec["name"]} '
            f'-x {spec["x"]} -y {spec["y"]} -z {spec["z"]} && exit 0; '
            'sleep 1; done; exit 1',
        ],
        name=f'spawn_{spec["name"]}',
        output='screen',
    )


def _setup(context, *args, **kwargs):
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    camera_preset = LaunchConfiguration('camera_preset').perform(context)

    if camera_preset not in ('forward', 'down'):
        raise RuntimeError(
            f"unknown camera_preset '{camera_preset}', expected 'forward' or 'down'")

    description_share = get_package_share_directory('npu_sim_description')
    bringup_share = get_package_share_directory('npu_sim_bringup')
    world_path = os.path.join(description_share, 'worlds', 'course_world.sdf')
    bridges_yaml = os.path.join(bringup_share, 'config', 'bridges.yaml')
    robot_model = os.path.join(description_share, 'models', 'robot', 'model.sdf')
    drone_model = _render_drone_model(camera_preset)

    server_only = headless or not gui
    gz_cmd = ['gz', 'sim']
    if server_only:
        gz_cmd.append('-s')
    gz_cmd += ['-r', world_path]
    mode = 'headless server' if server_only else 'GUI'

    gazebo = ExecuteProcess(
        cmd=gz_cmd,
        name='gazebo',
        output='screen',
        on_exit=ShutdownAction(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='npu_parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridges_yaml,
            'expand_gz_topic_names': True,
        }],
    )

    wait_clock = Node(
        package='npu_sim_bringup',
        executable='wait_for_clock',
        name='wait_for_clock',
        output='screen',
        arguments=['--timeout', CLOCK_TIMEOUT_SEC],
    )

    drone_adapter = Node(
        package='npu_sim_bringup',
        executable='drone_adapter',
        name='drone_adapter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    def on_clock_wait_exit(event, _context):
        if event.returncode == 0:
            return [drone_adapter]
        return [
            LogInfo(msg='fatal: /clock unavailable, shutting down bringup'),
            EmitEvent(event=Shutdown(reason='clock_wait_failed')),
        ]

    def on_spawn_exit(event, _context):
        if not event.process_name.startswith('spawn_'):
            return []
        if event.returncode != 0:
            return [
                LogInfo(msg=f'fatal: entity spawn failed ({event.process_name})'),
                EmitEvent(event=Shutdown(reason='spawn_failed')),
            ]
        return []

    return [
        LogInfo(msg=f'starting course simulation in {mode} mode, camera_preset={camera_preset}'),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(description_share, 'models'),
        ),
        gazebo,
        bridge,
        wait_clock,
        _spawn_action(robot_model, ROBOT_SPAWN),
        _spawn_action(drone_model, DRONE_SPAWN),
        RegisterEventHandler(OnProcessExit(
            target_action=wait_clock,
            on_exit=on_clock_wait_exit,
        )),
        RegisterEventHandler(OnProcessExit(on_exit=on_spawn_exit)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo GUI client. Ignored when headless:=true.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run the Gazebo server only, no GUI client.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Adapters timestamp from /clock when true.'),
        DeclareLaunchArgument(
            'camera_preset', default_value='forward',
            description="Camera mount preset: 'forward' or 'down'."),
        OpaqueFunction(function=_setup),
    ])
