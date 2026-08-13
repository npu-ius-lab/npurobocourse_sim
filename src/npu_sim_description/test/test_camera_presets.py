import math
import os
import xml.etree.ElementTree as ET

import pytest
import yaml


PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PRESETS_DIR = os.path.join(PKG_DIR, 'config', 'camera_presets')
TEMPLATE_PATH = os.path.join(PKG_DIR, 'models', 'drone', 'model.sdf.template')
DEFAULT_MODEL_PATH = os.path.join(PKG_DIR, 'models', 'drone', 'model.sdf')

# Gazebo camera sensors look along the +X axis of the sensor frame.
OPTICAL_AXIS = (1.0, 0.0, 0.0)


def _load_preset(name):
    with open(os.path.join(PRESETS_DIR, name + '.yaml')) as f:
        data = yaml.safe_load(f)
    return data['translation'], data['rpy']


def render_drone_model(translation, rpy):
    with open(TEMPLATE_PATH) as f:
        template = f.read()
    xyz = ' '.join(f'{v:.10g}' for v in translation)
    rpy_str = ' '.join(f'{v:.10g}' for v in rpy)
    rendered = template.replace('{{ camera_xyz }}', xyz)
    rendered = rendered.replace('{{ camera_rpy }}', rpy_str)
    assert '{{' not in rendered, 'unresolved template token remains'
    return rendered


def _rotation_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # ZYX intrinsic convention used by SDF poses
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _apply(matrix, vector):
    return tuple(sum(matrix[row][col] * vector[col] for col in range(3))
                 for row in range(3))


def _camera_pose(rendered_sdf):
    root = ET.fromstring(rendered_sdf)
    for sensor in root.iter('sensor'):
        if sensor.get('name') == 'front_cam':
            pose = sensor.findtext('pose').split()
            return [float(v) for v in pose]
    raise AssertionError('front_cam sensor not found')


def test_presets_declare_expected_mount():
    translation, rpy = _load_preset('forward')
    assert translation == pytest.approx([0.16, 0.0, -0.25])
    assert rpy == pytest.approx([0.0, math.radians(15), 0.0])

    translation, rpy = _load_preset('down')
    assert translation == pytest.approx([0.16, 0.0, -0.25])
    assert rpy == pytest.approx([0.0, math.radians(90), 0.0])


@pytest.mark.parametrize('preset', ['forward', 'down'])
def test_rendered_camera_pose_matches_preset(preset):
    translation, rpy = _load_preset(preset)
    rendered = render_drone_model(translation, rpy)
    pose = _camera_pose(rendered)
    assert pose[0:3] == pytest.approx(translation, abs=1e-6)
    assert pose[3:6] == pytest.approx(rpy, abs=1e-6)


def test_forward_preset_optical_transform():
    _, rpy = _load_preset('forward')
    axis = _apply(_rotation_matrix(*rpy), OPTICAL_AXIS)
    assert axis[0] == pytest.approx(math.cos(math.radians(15)), abs=1e-3)
    assert axis[1] == pytest.approx(0.0, abs=1e-3)
    assert axis[2] == pytest.approx(-math.sin(math.radians(15)), abs=1e-3)


def test_down_preset_optical_transform():
    _, rpy = _load_preset('down')
    axis = _apply(_rotation_matrix(*rpy), OPTICAL_AXIS)
    assert axis[0] == pytest.approx(0.0, abs=1e-3)
    assert axis[1] == pytest.approx(0.0, abs=1e-3)
    assert axis[2] == pytest.approx(-1.0, abs=1e-3)


def test_checked_in_default_model_matches_forward_preset():
    translation, rpy = _load_preset('forward')
    rendered = render_drone_model(translation, rpy)
    with open(DEFAULT_MODEL_PATH) as f:
        default_model = f.read()
    assert default_model == rendered, (
        'models/drone/model.sdf is out of sync with model.sdf.template; '
        'regenerate it from the forward preset')
