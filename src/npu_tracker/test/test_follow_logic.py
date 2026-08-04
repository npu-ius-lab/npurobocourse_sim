import math

from npu_tracker.follow_logic import DirectFollower, FollowConfig


def _follower(**kwargs):
    return DirectFollower(FollowConfig(**kwargs))


def test_moves_toward_offset_target():
    follower = _follower(follow_distance=1.5)
    cmd = follower.update((3.0, 0.0))
    assert cmd.linear.x > 0.0
    assert cmd.angular.z == 0.0


def test_backs_up_when_too_close():
    follower = _follower(follow_distance=1.5)
    cmd = follower.update((0.5, 0.0))
    assert cmd.linear.x < 0.0


def test_stops_inside_goal_tolerance():
    follower = _follower(follow_distance=1.5, goal_tolerance=0.15)
    cmd = follower.update((1.6, 0.0))
    assert cmd.linear.x == 0.0


def test_configurable_offset_changes_target():
    rel = (1.0, 0.0)
    far = _follower(follow_distance=0.5)
    near = _follower(follow_distance=1.5)
    assert far.update(rel).linear.x > 0.0
    assert near.update(rel).linear.x < 0.0


def test_linear_speed_bounded():
    follower = _follower(max_linear=0.5, linear_gain=0.8)
    cmd = follower.update((100.0, 0.0))
    assert cmd.linear.x <= 0.5 + 1e-9
    cmd = follower.update((-100.0, 0.0))
    assert abs(cmd.linear.x) <= 0.5 + 1e-9


def test_angular_rate_bounded_and_turns_toward_drone():
    follower = _follower(max_angular=1.0, angular_gain=1.5)
    cmd = follower.update((1.0, 5.0))
    assert abs(cmd.angular.z) <= 1.0 + 1e-9
    assert cmd.angular.z > 0.0
    cmd = follower.update((1.0, -5.0))
    assert cmd.angular.z < 0.0


def test_turns_before_driving_when_drone_to_the_side():
    follower = _follower()
    cmd = follower.update((0.0, 2.0))
    assert abs(cmd.linear.x) < 1e-9
    assert cmd.angular.z > 0.0


def test_stale_input_commands_zero():
    follower = _follower()
    cmd = follower.update(None)
    assert cmd.linear.x == 0.0
    assert cmd.angular.z == 0.0


def test_non_finite_input_commands_zero():
    follower = _follower()
    cmd = follower.update((math.nan, 0.0))
    assert cmd.linear.x == 0.0
    assert cmd.angular.z == 0.0
    cmd = follower.update((0.0, math.inf))
    assert cmd.linear.x == 0.0
    assert cmd.angular.z == 0.0


def test_zero_distance_commands_zero():
    follower = _follower()
    cmd = follower.update((0.0, 0.0))
    assert cmd.linear.x == 0.0
    assert cmd.angular.z == 0.0
