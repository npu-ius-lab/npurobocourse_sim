import math

import pytest

from npu_tracker.follow_goal_logic import (
    FollowGoalConfig,
    GoalReplacer,
    project_target,
)


def test_project_target_stops_follow_distance_short_of_drone():
    x, y, yaw = project_target((5.0, 5.0), (0.0, 0.0), follow_distance=1.5)
    dist_to_drone = math.hypot(5.0 - x, 5.0 - y)
    assert dist_to_drone == pytest.approx(1.5)
    # Target lies on the drone-to-robot ray.
    assert x == pytest.approx(y)
    assert 0.0 < x < 5.0
    # Yaw faces the drone.
    assert yaw == pytest.approx(math.atan2(5.0, 5.0))


def test_project_target_configurable_distance():
    x, y, _ = project_target((5.0, 0.0), (0.0, 0.0), follow_distance=2.0)
    assert x == pytest.approx(3.0)
    assert y == pytest.approx(0.0)


def test_project_target_when_robot_closer_than_follow_distance():
    x, y, yaw = project_target((1.0, 0.0), (0.0, 0.0), follow_distance=1.5)
    assert (x, y) == pytest.approx((0.0, 0.0))
    assert yaw == pytest.approx(0.0)


def test_project_target_coincident_yaw_zero():
    assert project_target((1.0, 1.0), (1.0, 1.0)) == (1.0, 1.0, 0.0)


def test_first_target_sent_immediately():
    replacer = GoalReplacer(FollowGoalConfig())
    assert replacer.update((1.0, 2.0, 0.5), now=0.0) == (1.0, 2.0, 0.5)


def test_replacement_rate_limited():
    replacer = GoalReplacer(FollowGoalConfig(goal_update_interval_sec=2.0))
    target = replacer.update((1.0, 1.0, 0.0), now=0.0)
    replacer.on_sent(target, now=0.0)
    assert replacer.update((3.0, 3.0, 0.0), now=1.0) is None
    assert replacer.update((3.0, 3.0, 0.0), now=2.1) == (3.0, 3.0, 0.0)


def test_small_target_movement_not_replaced():
    replacer = GoalReplacer(FollowGoalConfig(target_update_distance=0.3))
    target = replacer.update((1.0, 1.0, 0.0), now=0.0)
    replacer.on_sent(target, now=0.0)
    assert replacer.update((1.1, 1.1, 0.0), now=10.0) is None
    assert replacer.update((2.0, 1.0, 0.0), now=10.0) == (2.0, 1.0, 0.0)


def test_rejected_goal_waits_retry_delay():
    replacer = GoalReplacer(FollowGoalConfig(
        goal_update_interval_sec=0.1, goal_retry_delay_sec=2.0))
    replacer.on_rejected_or_failed(now=5.0)
    assert replacer.update((1.0, 1.0, 0.0), now=6.0) is None
    assert replacer.update((1.0, 1.0, 0.0), now=7.1) == (1.0, 1.0, 0.0)


def test_backoff_resets_sent_history():
    replacer = GoalReplacer(FollowGoalConfig(
        target_update_distance=0.3, goal_update_interval_sec=0.1,
        goal_retry_delay_sec=1.0))
    target = replacer.update((1.0, 1.0, 0.0), now=0.0)
    replacer.on_sent(target, now=0.0)
    replacer.on_rejected_or_failed(now=1.0)
    # Same target may be resent after backoff: history was cleared.
    assert replacer.update((1.0, 1.0, 0.0), now=2.1) == (1.0, 1.0, 0.0)


def test_in_backoff():
    replacer = GoalReplacer(FollowGoalConfig(goal_retry_delay_sec=2.0))
    assert not replacer.in_backoff(now=0.0)
    replacer.on_rejected_or_failed(now=0.0)
    assert replacer.in_backoff(now=1.0)
    assert not replacer.in_backoff(now=2.1)
