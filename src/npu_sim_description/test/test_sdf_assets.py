import os
import xml.etree.ElementTree as ET

import pytest


PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS_DIR = os.path.join(PKG_DIR, 'models')
WORLDS_DIR = os.path.join(PKG_DIR, 'worlds')


def _parse(path):
    return ET.parse(path).getroot()


def test_course_world_parses_and_defines_course_elements():
    root = _parse(os.path.join(WORLDS_DIR, 'course_world.sdf'))
    assert root.tag == 'sdf'
    world = root.find('world')
    assert world is not None
    assert world.get('name') == 'course_world'

    plugin_files = [p.get('filename') for p in world.findall('plugin')]
    for required in ['gz-sim-physics-system', 'gz-sim-user-commands-system',
                     'gz-sim-scene-broadcaster-system', 'gz-sim-sensors-system',
                     'gz-sim-imu-system']:
        assert required in plugin_files

    includes = world.findall('include')
    ring = [i for i in includes if i.find('name') is not None
            and i.find('name').text == 'ring']
    assert len(ring) == 1, 'course_world must include exactly one ring model'
    assert ring[0].find('pose') is not None

    model_names = [m.get('name') for m in world.findall('model')]
    assert 'ground_plane' in model_names
    assert 'arena' in model_names
    assert sum(1 for n in model_names if n.startswith('obstacle_')) >= 1


def test_robot_model_structure():
    root = _parse(os.path.join(MODELS_DIR, 'robot', 'model.sdf'))
    model = root.find('model')
    assert model.get('name') == 'npu_robot'

    link_names = [l.get('name') for l in model.findall('link')]
    for required in ['base_link', 'left_wheel_link', 'right_wheel_link',
                     'caster_link', 'lidar_link']:
        assert required in link_names

    diff_drive = None
    for plugin in model.findall('plugin'):
        if plugin.get('filename') == 'gz-sim-diff-drive-system':
            diff_drive = plugin
    assert diff_drive is not None
    assert diff_drive.findtext('left_joint') == 'left_wheel_joint'
    assert diff_drive.findtext('right_joint') == 'right_wheel_joint'
    assert diff_drive.findtext('wheel_separation') is not None
    assert diff_drive.findtext('wheel_radius') is not None
    assert diff_drive.findtext('frame_id') == 'npu_robot/odom'
    assert diff_drive.findtext('child_frame_id') == 'npu_robot/base_link'

    lidar = None
    for link in model.findall('link'):
        if link.get('name') == 'lidar_link':
            lidar = link.find('sensor')
    assert lidar is not None
    assert lidar.get('type') == 'gpu_lidar'
    assert lidar.findtext('topic') == '/npu/robot/scan'

    marker_include = [i for i in model.findall('include')
                      if i.findtext('uri') == 'model://marker']
    assert len(marker_include) == 1
    assert marker_include[0].find('pose') is not None

    weld = [j for j in model.findall('joint')
            if j.get('type') == 'fixed'
            and j.findtext('child', '').startswith('marker::')]
    assert len(weld) == 1, 'marker must be rigidly welded to the robot'


def test_drone_model_plugins():
    root = _parse(os.path.join(MODELS_DIR, 'drone', 'model.sdf'))
    model = root.find('model')
    assert model.get('name') == 'npu_drone'

    motor_plugins = [p for p in model.findall('plugin')
                     if p.get('filename') == 'gz-sim-multicopter-motor-model-system']
    assert len(motor_plugins) == 4
    actuator_numbers = sorted(int(p.findtext('actuator_number'))
                              for p in motor_plugins)
    assert actuator_numbers == [0, 1, 2, 3]
    directions = [p.findtext('turningDirection') for p in motor_plugins]
    assert directions.count('ccw') == 2
    assert directions.count('cw') == 2

    vel = [p for p in model.findall('plugin')
           if p.get('filename') == 'gz-sim-multicopter-control-system']
    assert len(vel) == 1
    vel = vel[0]
    assert vel.findtext('commandSubTopic') == 'gazebo/command/twist'
    rotors = vel.find('rotorConfiguration').findall('rotor')
    assert len(rotors) == 4
    for rotor in rotors:
        assert rotor.findtext('jointName', '').startswith('rotor_')
        assert rotor.findtext('forceConstant') is not None
        assert rotor.findtext('momentConstant') is not None
        assert rotor.findtext('direction') in ('1', '-1')

    odom = [p for p in model.findall('plugin')
            if p.get('filename') == 'gz-sim-odometry-publisher-system']
    assert len(odom) == 1
    assert odom[0].findtext('odom_frame') == 'world'

    camera = None
    base = [l for l in model.findall('link') if l.get('name') == 'base_link'][0]
    for sensor in base.findall('sensor'):
        if sensor.get('name') == 'front_cam':
            camera = sensor
    assert camera is not None
    assert camera.get('type') == 'camera'
    assert camera.findtext('topic') == '/npu/drone/camera/image_raw'
    assert camera.find('pose') is not None


def test_marker_model_and_texture_exist():
    root = _parse(os.path.join(MODELS_DIR, 'marker', 'model.sdf'))
    model = root.find('model')
    mesh = model.find('.//mesh/uri')
    assert mesh is not None
    assert mesh.text.startswith('model://marker/')
    rel = mesh.text.replace('model://marker/', '')
    assert os.path.isfile(os.path.join(MODELS_DIR, 'marker', rel))
    png = os.path.join(MODELS_DIR, 'marker', 'meshes', 'tag36_11_00008.png')
    assert os.path.isfile(png)


@pytest.mark.parametrize('name', ['ring', 'robot', 'marker', 'drone', 'drone_display'])
def test_model_config_exists(name):
    path = os.path.join(MODELS_DIR, name, 'model.config')
    assert os.path.isfile(path)
    root = _parse(path)
    assert root.find('sdf') is not None
