import math

from npu_tracker.tag_tracker_logic import TagTracker, TagTrackerConfig


def _tracker(**overrides):
    config = TagTrackerConfig(**overrides)
    return TagTracker(config)


def test_zero_command_when_marker_missing():
    tracker = _tracker()
    cmd = tracker.update(None, (0.0, 0.0, 1.0, 0.0))
    assert cmd.linear.x == 0.0
    assert cmd.linear.y == 0.0
    assert cmd.linear.z == 0.0
    assert cmd.angular.z == 0.0


def test_zero_command_when_drone_pose_missing():
    tracker = _tracker()
    cmd = tracker.update((1.0, 0.0, 0.0), None)
    assert cmd.linear.x == 0.0
    assert cmd.linear.z == 0.0


def test_moves_toward_marker_in_world_x():
    tracker = _tracker()
    # Drone at origin hovering at the right height, marker 2 m ahead in +x.
    cmd = tracker.update((2.0, 0.0, 0.0), (0.0, 0.0, 1.2, 0.0))
    assert cmd.linear.x > 0.0
    assert abs(cmd.linear.y) < 1e-6
    assert abs(cmd.linear.z) < 1e-6


def test_climbs_to_hover_height():
    tracker = _tracker()
    # Drone directly above the marker but too low.
    cmd = tracker.update((0.0, 0.0, 0.0), (0.0, 0.0, 0.5, 0.0))
    assert cmd.linear.z > 0.0
    assert cmd.linear.z <= 0.5 + 1e-9


def test_descends_when_too_high():
    tracker = _tracker()
    cmd = tracker.update((0.0, 0.0, 0.0), (0.0, 0.0, 3.0, 0.0))
    assert cmd.linear.z < 0.0


def test_horizontal_speed_is_clamped():
    tracker = _tracker(max_linear_xy=0.3)
    cmd = tracker.update((50.0, 0.0, 0.0), (0.0, 0.0, 1.2, 0.0))
    assert cmd.linear.x <= 0.3 + 1e-9


def test_deadband_holds_position():
    tracker = _tracker()
    # Inside the tolerance: no horizontal or vertical motion.
    cmd = tracker.update((0.05, 0.0, 0.0), (0.0, 0.0, 1.2 + 0.05, 0.0))
    assert cmd.linear.x == 0.0
    assert cmd.linear.z == 0.0


def test_command_is_body_frame():
    tracker = _tracker()
    # Drone faces +y (yaw 90 deg); marker in world +x is body -y.
    cmd = tracker.update((2.0, 0.0, 0.0), (0.0, 0.0, 1.2, math.pi / 2.0))
    assert abs(cmd.linear.x) < 1e-6
    assert cmd.linear.y < 0.0


def test_yaws_toward_distant_marker():
    tracker = _tracker()
    cmd = tracker.update((0.0, 2.0, 0.0), (0.0, 0.0, 1.2, 0.0))
    assert cmd.angular.z > 0.0
    assert cmd.angular.z <= 0.8 + 1e-9
