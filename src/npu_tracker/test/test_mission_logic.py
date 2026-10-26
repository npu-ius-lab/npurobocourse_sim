import math

from npu_tracker.mission_logic import (
    APPROACH_RING,
    APPROACH_WAYPOINT,
    COMPLETE,
    FAILED,
    TAKEOFF,
    TRAVERSE_RING,
    Mission,
    MissionConfig,
    world_to_body,
    wrap_angle,
)


def _mission(**kwargs):
    config = MissionConfig(**kwargs)
    return Mission(config)


def _pose(x, y, z, yaw=0.0):
    return (x, y, z, yaw)


def test_world_to_body_rotation():
    vx, vy = world_to_body(1.0, 0.0, math.pi / 2)
    assert abs(vx) < 1e-9
    assert abs(vy + 1.0) < 1e-9
    vx, vy = world_to_body(0.0, 1.0, math.pi)
    assert abs(vx) < 1e-9
    assert abs(vy + 1.0) < 1e-9


def test_wrap_angle():
    assert abs(wrap_angle(3 * math.pi) - math.pi) < 1e-9
    assert abs(wrap_angle(-3 * math.pi) + math.pi) < 1e-9
    assert wrap_angle(0.5) == 0.5


def test_idle_to_takeoff_on_first_pose():
    mission = _mission()
    cmd, state = mission.update(None, now=0.0)
    assert state == 'IDLE'
    assert cmd.linear.x == 0.0
    cmd, state = mission.update(_pose(-1.5, -4.0, 0.32), now=0.1)
    assert state == TAKEOFF
    assert cmd.linear.z > 0.0


def test_takeoff_completion_advances_to_ring():
    mission = _mission(takeoff_height=1.0)
    mission.update(_pose(-1.5, -4.0, 0.32), now=0.0)
    cmd, state = mission.update(_pose(-1.5, -4.0, 1.0), now=1.0)
    assert state == APPROACH_RING
    assert cmd.linear.y > 0.0


def test_full_mission_reaches_complete():
    mission = _mission(
        takeoff_height=1.0,
        ring_approach=(-1.0, 2.8, 1.0),
        ring_exit=(-1.0, 5.8, 1.0),
        waypoints=[(2.5, 2.0, 1.0)],
    )
    mission.update(_pose(-1.5, -4.0, 0.32), now=0.0)
    mission.update(_pose(-1.5, -4.0, 1.0), now=1.0)
    _, state = mission.update(_pose(-1.0, 2.8, 1.0), now=2.0)
    assert state == TRAVERSE_RING
    _, state = mission.update(_pose(-1.0, 5.8, 1.0), now=3.0)
    assert state == APPROACH_WAYPOINT
    cmd, state = mission.update(_pose(2.5, 2.0, 1.0), now=4.0)
    assert state == COMPLETE
    assert cmd.linear.x == 0.0
    assert cmd.linear.y == 0.0
    assert cmd.linear.z == 0.0
    assert cmd.angular.z == 0.0
    cmd, state = mission.update(_pose(3.0, 3.0, 2.0), now=5.0)
    assert state == COMPLETE
    assert cmd.linear.z == 0.0


def test_next_waypoint_resets_state_timeout():
    mission = _mission(
        state_timeout_sec=5.0,
        waypoints=[(2.5, 2.0, 1.0), (4.0, 2.0, 1.0)],
    )
    mission.update(_pose(-1.5, -4.0, 0.32), now=0.0)
    mission.update(_pose(-1.5, -4.0, 1.0), now=1.0)
    mission.update(_pose(-1.0, 2.8, 1.0), now=2.0)
    mission.update(_pose(-1.0, 5.8, 1.0), now=3.0)
    _, state = mission.update(_pose(2.5, 2.0, 1.0), now=4.0)
    assert state == APPROACH_WAYPOINT
    cmd, state = mission.update(_pose(4.0, 2.0, 1.0), now=8.5)
    assert state == COMPLETE
    assert cmd.linear.x == 0.0
    assert cmd.linear.y == 0.0
    assert cmd.linear.z == 0.0


def test_state_timeout_goes_failed_and_zeroes():
    mission = _mission(state_timeout_sec=10.0)
    mission.update(_pose(-1.5, -4.0, 0.32), now=0.0)
    cmd, state = mission.update(_pose(-1.5, -4.0, 0.5), now=11.0)
    assert state == FAILED
    assert cmd.linear.z == 0.0
    cmd, state = mission.update(_pose(-1.5, -4.0, 1.0), now=12.0)
    assert state == FAILED
    assert cmd.linear.z == 0.0


def test_stale_pose_zeroes_command():
    mission = _mission()
    mission.update(_pose(-1.5, -4.0, 0.32), now=0.0)
    cmd, state = mission.update(None, now=1.0)
    assert state == TAKEOFF
    assert cmd.linear.z == 0.0


def test_horizontal_speed_clamped_to_cruise():
    mission = _mission(cruise_speed=0.8, approach_distance_gain=1.0)
    mission.update(_pose(-1.5, -4.0, 0.32), now=0.0)
    mission.update(_pose(-1.5, -4.0, 1.0), now=0.1)
    cmd, _ = mission.update(_pose(-1.5, -4.0, 1.0, 0.0), now=0.2)
    speed = math.hypot(cmd.linear.x, cmd.linear.y)
    assert speed <= 0.8 + 1e-9


def test_vertical_speed_clamped():
    mission = _mission(vertical_speed=0.5)
    mission.update(_pose(-1.5, -4.0, 0.32), now=0.0)
    cmd, _ = mission.update(_pose(-1.5, -4.0, 0.0), now=0.1)
    assert cmd.linear.z <= 0.5 + 1e-9


def test_yaw_rate_limited_and_turns_toward_target():
    mission = _mission(yaw_rate_limit=0.8)
    mission.update(_pose(-1.5, -4.0, 0.32), now=0.0)
    mission.update(_pose(-1.5, -4.0, 1.0), now=0.1)
    cmd, _ = mission.update(_pose(-1.5, -4.0, 1.0, math.pi), now=0.2)
    assert abs(cmd.angular.z) <= 0.8 + 1e-9
    assert cmd.angular.z != 0.0


def test_body_frame_command_with_yaw():
    mission = _mission(ring_approach=(1.0, 0.0, 1.0), cruise_speed=1.0,
                       approach_distance_gain=2.0)
    mission.update(_pose(0.0, 0.0, 0.0), now=0.0)
    mission.update(_pose(0.0, 0.0, 1.0), now=0.1)
    cmd, _ = mission.update(_pose(0.0, 0.0, 1.0, math.pi / 2), now=0.2)
    assert abs(cmd.linear.x) < 1e-9
    assert cmd.linear.y < 0.0
