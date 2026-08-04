import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class FollowGoalConfig:
    # Desired distance between robot and drone; the nav target stops this
    # far short of the drone along the drone-to-robot ray so the goal cell
    # stays reachable when the drone hovers near obstacles.
    follow_distance: float = 1.5
    # Minimum xy displacement of the projected target before a replacement
    # goal is worth sending.
    target_update_distance: float = 0.3
    # Minimum wall-time between two goal submissions (rate limit).
    goal_update_interval_sec: float = 2.0
    # Backoff after a rejected or aborted goal before the next submission.
    goal_retry_delay_sec: float = 2.0


def project_target(
    drone_xy: Tuple[float, float],
    robot_xy: Tuple[float, float],
    follow_distance: float = 1.5,
) -> Tuple[float, float, float]:
    """Project the drone ground position into a world-frame nav target.

    Returns (x, y, yaw): the offset point follow_distance behind the drone
    on the drone-to-robot ray, with yaw facing the drone.
    """
    dx = robot_xy[0] - drone_xy[0]
    dy = robot_xy[1] - drone_xy[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return (drone_xy[0], drone_xy[1], 0.0)
    scale = min(follow_distance, dist) / dist
    x = drone_xy[0] + dx * scale
    y = drone_xy[1] + dy * scale
    yaw = math.atan2(-dy, -dx)
    return (x, y, yaw)


class GoalReplacer:
    """Decides when an active NavigateToPose goal may be replaced.

    update() returns the target to submit, or None to keep waiting. The
    caller must cancel the active goal before submitting a replacement.
    Rejected or aborted goals trigger a retry backoff so failures never
    spin in an immediate resend loop.
    """

    def __init__(self, config: FollowGoalConfig):
        self.config = config
        self._last_target: Optional[Tuple[float, float]] = None
        self._last_send_time: Optional[float] = None
        self._retry_not_before: float = 0.0

    def update(self, target: Tuple[float, float, float], now: float) -> Optional[Tuple[float, float, float]]:
        if now < self._retry_not_before:
            return None
        if self._last_send_time is not None:
            if now - self._last_send_time < self.config.goal_update_interval_sec:
                return None
        xy = (target[0], target[1])
        if self._last_target is not None:
            moved = math.hypot(xy[0] - self._last_target[0],
                               xy[1] - self._last_target[1])
            if moved < self.config.target_update_distance:
                return None
        return target

    def on_sent(self, target: Tuple[float, float, float], now: float) -> None:
        self._last_target = (target[0], target[1])
        self._last_send_time = now

    def on_rejected_or_failed(self, now: float) -> None:
        self._retry_not_before = now + self.config.goal_retry_delay_sec
        self._last_target = None
        self._last_send_time = None

    def in_backoff(self, now: float) -> bool:
        return now < self._retry_not_before
