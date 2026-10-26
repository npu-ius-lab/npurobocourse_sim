import ast
import os

import pytest
import yaml


PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DISPLAY_CLASSES = {
    'Grid': 'rviz_default_plugins/Grid',
    'Map': 'rviz_default_plugins/Map',
    'LaserScan': 'rviz_default_plugins/LaserScan',
    'TF': 'rviz_default_plugins/TF',
}


def _read(*parts):
    with open(os.path.join(PKG_DIR, *parts)) as f:
        return f.read()


def _load_rviz(name):
    return yaml.safe_load(_read('config', name + '.rviz'))


@pytest.mark.parametrize('name', ['amcl', 'slam'])
def test_rviz_has_teaching_displays(name):
    config = _load_rviz(name)
    manager = config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'world'
    displays = {display['Name']: display for display in manager['Displays']}
    for display_name, plugin_name in DISPLAY_CLASSES.items():
        assert displays[display_name]['Class'] == plugin_name
    assert displays['Map']['Topic']['Value'] == '/map'
    assert displays['LaserScan']['Topic']['Value'] == '/npu/robot/scan'


def test_amcl_has_navigation_displays():
    config = _load_rviz('amcl')
    displays = {
        display['Class']: display
        for display in config['Visualization Manager']['Displays']
    }
    expected = {
        'rviz_default_plugins/PoseArray': '/particlecloud',
        'rviz_default_plugins/Path': '/plan',
        'rviz_default_plugins/Odometry': '/npu/robot/odom',
    }
    for plugin_name, topic_name in expected.items():
        display = displays[plugin_name]
        assert display['Enabled'] is True
        assert display['Value'] is True
        assert display['Topic'] == {
            'Depth': 5,
            'Durability Policy': 'Volatile',
            'History Policy': 'Keep Last',
            'Reliability Policy': 'Reliable',
            'Value': topic_name,
        }
    assert displays['rviz_default_plugins/Path']['Buffer Length'] == 1
    assert displays['rviz_default_plugins/Odometry']['Keep'] >= 100


def test_amcl_has_initial_pose_and_nav2_goal_tools():
    config = _load_rviz('amcl')
    tools = {tool['Class']: tool
             for tool in config['Visualization Manager']['Tools']}
    initial_pose = tools['rviz_default_plugins/SetInitialPose']
    assert initial_pose['Topic']['Value'] == '/initialpose'
    assert 'nav2_rviz_plugins/GoalTool' in tools


def test_slam_keeps_toolbox_panel():
    config = _load_rviz('slam')
    panels = {panel['Class'] for panel in config['Panels']}
    assert 'slam_toolbox::SlamToolboxPlugin' in panels


@pytest.mark.parametrize(('launch_name', 'upstream_package'), [
    ('amcl', 'nav2_bringup'),
    ('slam', 'slam_toolbox'),
])
def test_launch_uses_local_rviz_config(launch_name, upstream_package):
    source = _read('launch', launch_name + '.launch.py')
    expected = f"os.path.join(pkg_share, 'config', '{launch_name}.rviz')"
    assert expected in source
    assert f"get_package_share_directory('{upstream_package}')" not in source


def test_behavior_server_uses_controller_cmd_vel_remap():
    tree = ast.parse(_read('launch', 'amcl.launch.py'))
    assignment = next(
        node for node in tree.body
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == 'NAV2_NODES'
                for target in node.targets))
    nodes = {name: remappings for _, _, name, remappings
             in ast.literal_eval(assignment.value)}
    expected = {'/cmd_vel': '/npu/robot/cmd_vel'}
    assert nodes['controller_server'] == expected
    assert nodes['behavior_server'] == expected
