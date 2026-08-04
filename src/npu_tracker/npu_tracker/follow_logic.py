import math
from dataclasses import dataclass
from typing import Optional, Tuple

from geometry_msgs.msg import Twist

from npu_tracker.mission_logic import _clamp


@dataclass
class FollowConfig:
    follow_distance: float = 1.5
    goal_tolerance: float = 0.15
    max_linear: float = 0.5
    max_angular: float = 1.0
    linear_gain: float = 0.8
    angular_gain: float = 1.5


class DirectFollower:
    """Direct (no obstacle avoidance) drone-following controller.

    update() consumes the drone xy position expressed in the robot base
    frame, or None when the input pose/TF is missing or stale, and returns
    the bounded velocity command to publish on /npu/robot/cmd_vel. The
    offset target sits follow_distance behind the drone along the
    robot-to-drone line; the robot stops inside goal_tolerance around it.
    """

    def __init__(self, config: FollowConfig):
        self.config = config

    def update(self, rel: Optional[Tuple[float, float]]) -> Twist:
        cmd = Twist()
        if rel is None:
            return cmd

        dx, dy = rel
        if not (math.isfinite(dx) and math.isfinite(dy)):
            return cmd

        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return cmd

        distance_err = dist - self.config.follow_distance
        angle_err = math.atan2(dy, dx)

        cmd.angular.z = _clamp(
            self.config.angular_gain * angle_err, self.config.max_angular)

        linear = self.config.linear_gain * distance_err
        linear *= max(0.0, math.cos(angle_err))
        if abs(distance_err) <= self.config.goal_tolerance:
            linear = 0.0
        cmd.linear.x = _clamp(linear, self.config.max_linear)

        return cmd
