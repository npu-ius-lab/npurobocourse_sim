import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from geometry_msgs.msg import Twist


IDLE = 'IDLE'
TAKEOFF = 'TAKEOFF'
APPROACH_RING = 'APPROACH_RING'
TRAVERSE_RING = 'TRAVERSE_RING'
APPROACH_WAYPOINT = 'APPROACH_WAYPOINT'
COMPLETE = 'COMPLETE'
FAILED = 'FAILED'

TERMINAL_STATES = (COMPLETE, FAILED)


@dataclass
class MissionConfig:
    takeoff_height: float = 1.0
    ring_approach: Tuple[float, float, float] = (-1.0, 2.8, 1.0)
    ring_exit: Tuple[float, float, float] = (-1.0, 5.8, 1.0)
    waypoints: List[Tuple[float, float, float]] = field(
        default_factory=lambda: [(2.5, 2.0, 1.0)])
    position_tolerance: float = 0.25
    cruise_speed: float = 0.8
    vertical_speed: float = 0.5
    yaw_rate_limit: float = 0.8
    approach_distance_gain: float = 1.0
    state_timeout_sec: float = 120.0


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def world_to_body(vx_w: float, vy_w: float, yaw: float) -> Tuple[float, float]:
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    return (
        cos_y * vx_w + sin_y * vy_w,
        -sin_y * vx_w + cos_y * vy_w,
    )


class Mission:
    """Ring/waypoint flight state machine.

    update() consumes the current world-frame pose (x, y, z, yaw) or None when
    no fresh pose is available, and returns the body-frame velocity command to
    publish. Terminal states always return a zero command.
    """

    def __init__(self, config: MissionConfig):
        self.config = config
        self.state = IDLE
        self._state_start: Optional[float] = None
        self._waypoint_index = 0
        self._home: Optional[Tuple[float, float, float]] = None

    def _enter(self, state: str, now: float) -> None:
        self.state = state
        self._state_start = now

    def _current_target(self) -> Optional[Tuple[float, float, float]]:
        if self.state == TAKEOFF:
            return (self._home[0], self._home[1], self.config.takeoff_height)
        if self.state == APPROACH_RING:
            return self.config.ring_approach
        if self.state == TRAVERSE_RING:
            return self.config.ring_exit
        if self.state == APPROACH_WAYPOINT:
            return self.config.waypoints[self._waypoint_index]
        return None

    def _advance(self, now: float) -> None:
        if self.state == TAKEOFF:
            self._enter(APPROACH_RING, now)
        elif self.state == APPROACH_RING:
            self._enter(TRAVERSE_RING, now)
        elif self.state == TRAVERSE_RING:
            self._waypoint_index = 0
            self._enter(APPROACH_WAYPOINT, now)
        elif self.state == APPROACH_WAYPOINT:
            self._waypoint_index += 1
            if self._waypoint_index >= len(self.config.waypoints):
                self._enter(COMPLETE, now)
            else:
                # 每个航点单独计时，避免上一航点耗时导致下一航点误超时。
                self._enter(APPROACH_WAYPOINT, now)

    def update(
        self,
        pose: Optional[Tuple[float, float, float, float]],
        now: float,
    ) -> Tuple[Twist, str]:
        zero = Twist()

        if self.state in TERMINAL_STATES:
            return zero, self.state

        if self.state == IDLE:
            if pose is None:
                return zero, self.state
            self._home = (pose[0], pose[1], pose[2])
            self._enter(TAKEOFF, now)

        if self._state_start is not None and now - self._state_start > self.config.state_timeout_sec:
            self._enter(FAILED, now)
            return zero, self.state

        if pose is None:
            return zero, self.state

        x, y, z, yaw = pose
        target = self._current_target()
        err_x = target[0] - x
        err_y = target[1] - y
        err_z = target[2] - z
        dist_xy = math.hypot(err_x, err_y)
        dist_3d = math.sqrt(dist_xy ** 2 + err_z ** 2)

        if dist_3d < self.config.position_tolerance:
            self._advance(now)
            if self.state in TERMINAL_STATES:
                return zero, self.state
            target = self._current_target()
            err_x = target[0] - x
            err_y = target[1] - y
            err_z = target[2] - z
            dist_xy = math.hypot(err_x, err_y)

        speed = min(self.config.cruise_speed,
                    self.config.approach_distance_gain * dist_xy)
        cmd = Twist()
        if dist_xy > 1e-3:
            vx_w = speed * err_x / dist_xy
            vy_w = speed * err_y / dist_xy
        else:
            vx_w = 0.0
            vy_w = 0.0
        cmd.linear.x, cmd.linear.y = world_to_body(vx_w, vy_w, yaw)
        cmd.linear.z = _clamp(err_z, self.config.vertical_speed)

        if dist_xy > 0.3:
            desired_yaw = math.atan2(err_y, err_x)
            yaw_err = wrap_angle(desired_yaw - yaw)
            cmd.angular.z = _clamp(1.5 * yaw_err, self.config.yaw_rate_limit)

        return cmd, self.state
