import os

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


DISPLAY_SPAWN = {'name': 'npu_drone_display', 'x': '0.0', 'y': '0.0', 'z': '0.32'}
WORLD_NAME = 'course_world'


def _spawn_action(model_file, spec, attempts=60):
    return ExecuteProcess(
        cmd=[
            'bash', '-c',
            'created=0; '
            f'for i in $(seq 1 {attempts}); do '
            'ros2 run ros_gz_sim create '
            f'-world {WORLD_NAME} -file {model_file} -name {spec["name"]} '
            f'-x {spec["x"]} -y {spec["y"]} -z {spec["z"]} '
            '&& created=1 && break; '
            'sleep 1; done; '
            '[ "$created" -eq 1 ] || exit 1; '
            f'for i in $(seq 1 {attempts}); do '
            f'pose=$(gz model -m {spec["name"]} --pose 2>/dev/null) && '
            '[[ "$pose" == *Pose* ]] && exit 0; '
            'sleep 1; done; exit 1',
        ],
        name=f'spawn_{spec["name"]}',
        output='screen',
    )


def _setup(context, *args, **kwargs):
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    description_share = get_package_share_directory('npu_sim_description')
    bringup_share = get_package_share_directory('npu_sim_bringup')
    world_path = os.path.join(description_share, 'worlds', 'course_world.sdf')
    display_yaml = os.path.join(bringup_share, 'config', 'display.yaml')
    display_model = os.path.join(
        description_share, 'models', 'drone_display', 'model.sdf')

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

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='npu_clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    adapter = Node(
        package='npu_sim_bringup',
        executable='external_pose_display_adapter',
        name='external_pose_display_adapter',
        output='screen',
        parameters=[display_yaml, {'use_sim_time': use_sim_time}],
    )

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
        LogInfo(msg=f'starting virtual-real display scene in {mode} mode'),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(description_share, 'models'),
        ),
        gazebo,
        clock_bridge,
        _spawn_action(display_model, DISPLAY_SPAWN),
        adapter,
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
        OpaqueFunction(function=_setup),
    ])
