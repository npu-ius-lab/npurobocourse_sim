import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


NAV2_NODES = [
    ('nav2_map_server', 'map_server', 'map_server', {}),
    ('nav2_amcl', 'amcl', 'amcl', {}),
    ('nav2_controller', 'controller_server', 'controller_server',
     {'/cmd_vel': '/npu/robot/cmd_vel'}),
    ('nav2_planner', 'planner_server', 'planner_server', {}),
    ('nav2_behaviors', 'behavior_server', 'behavior_server',
     {'/cmd_vel': '/npu/robot/cmd_vel'}),
    ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', {}),
]


def generate_launch_description():
    pkg_share = get_package_share_directory('npu_mobile_robot')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2.yaml')
    default_map = os.path.join(pkg_share, 'maps', 'course_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')

    robot_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'robot_tf.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    nav2_nodes = []
    for package, executable, name, remappings in NAV2_NODES:
        parameters = [nav2_params, {'use_sim_time': use_sim_time}]
        if name == 'map_server':
            parameters.append({'yaml_filename': map_yaml})
        nav2_nodes.append(Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            parameters=parameters,
            remappings=list(remappings.items()),
        ))

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    rviz_node = Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'amcl.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time from /clock.'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Full path to the occupancy map YAML served by map_server.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Open RViz with the course AMCL view.'),
        robot_tf,
        *nav2_nodes,
        lifecycle_manager,
        rviz_node,
    ])
