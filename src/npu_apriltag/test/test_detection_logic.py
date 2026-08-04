import math
from types import SimpleNamespace

import pytest

from npu_apriltag.detection_logic import (
    mount_quaternion,
    quat_from_rpy,
    quat_multiply,
    quat_rotate,
    select_detection,
)


def test_quat_from_rpy_identity():
    q = quat_from_rpy(0.0, 0.0, 0.0)
    assert q == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_quat_rotate_yaw_90():
    q = quat_from_rpy(0.0, 0.0, math.pi / 2)
    x, y, z = quat_rotate(q, (1.0, 0.0, 0.0))
    assert (x, y, z) == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)


def test_quat_multiply_composes_rotations():
    yaw90 = quat_from_rpy(0.0, 0.0, math.pi / 2)
    combined = quat_multiply(yaw90, yaw90)
    x, y, z = quat_rotate(combined, (1.0, 0.0, 0.0))
    assert (x, y, z) == pytest.approx((-1.0, 0.0, 0.0), abs=1e-9)


def test_forward_mount_points_camera_15_deg_down():
    q = mount_quaternion((0.0, math.radians(15), 0.0))
    forward = quat_rotate(q, (0.0, 0.0, 1.0))
    assert forward == pytest.approx(
        (math.cos(math.radians(15)), 0.0, -math.sin(math.radians(15))), abs=1e-9)


def test_down_mount_points_camera_straight_down():
    q = mount_quaternion((0.0, math.radians(90), 0.0))
    forward = quat_rotate(q, (0.0, 0.0, 1.0))
    assert forward == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)


def test_optical_x_is_image_right():
    q = mount_quaternion((0.0, 0.0, 0.0))
    right = quat_rotate(q, (1.0, 0.0, 0.0))
    assert right == pytest.approx((0.0, -1.0, 0.0), abs=1e-9)


def test_select_detection_picks_configured_id():
    detections = [SimpleNamespace(id=3), SimpleNamespace(id=8), SimpleNamespace(id=1)]
    selected = select_detection(detections, 8)
    assert selected is detections[1]


def test_select_detection_missing_marker():
    detections = [SimpleNamespace(id=3), SimpleNamespace(id=1)]
    assert select_detection(detections, 8) is None
    assert select_detection([], 8) is None
