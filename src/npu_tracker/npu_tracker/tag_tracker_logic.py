import math
from dataclasses import dataclass
from typing import Optional, Tuple

from geometry_msgs.msg import Twist

from npu_tracker.mission_logic import _clamp, world_to_body, wrap_angle


@dataclass
class TagTrackerConfig:
    hover_height: float = 1.2
    position_tolerance: float = 0.15
    max_linear_xy: float = 0.8
    max_vertical: float = 0.5
    yaw_rate_limit: float = 0.8
    distance_gain: float = 1.0
    face_marker_distance: float = 0.5


class TagTracker:
    """Marker-hover controller.

    update() consumes the selected marker position in the world frame
    (x, y, z) or None, and the current drone pose (x, y, z, yaw) or None,
    and returns the body-frame velocity command to publish. Missing or
    stale inputs always produce a zero command.
    """

    def __init__(self, config: TagTrackerConfig):
        self.config = config

    def update(
        self,
        marker: Optional[Tuple[float, float, float]],
        pose: Optional[Tuple[float, float, float, float]],
    ) -> Twist:
        cmd = Twist()
        if marker is None or pose is None:
            return cmd

        x, y, z, yaw = pose
        target = (marker[0], marker[1], marker[2] + self.config.hover_height)
        err_x = target[0] - x
        err_y = target[1] - y
        err_z = target[2] - z
        dist_xy = math.hypot(err_x, err_y)

        speed = min(self.config.max_linear_xy,
                    self.config.distance_gain * dist_xy)
        if dist_xy > self.config.position_tolerance and dist_xy > 1e-3:
            vx_w = speed * err_x / dist_xy
            vy_w = speed * err_y / dist_xy
        else:
            vx_w = 0.0
            vy_w = 0.0
        cmd.linear.x, cmd.linear.y = world_to_body(vx_w, vy_w, yaw)

        if abs(err_z) > self.config.position_tolerance:
            cmd.linear.z = _clamp(err_z, self.config.max_vertical)

        if dist_xy > self.config.face_marker_distance:
            desired_yaw = math.atan2(err_y, err_x)
            yaw_err = wrap_angle(desired_yaw - yaw)
            cmd.angular.z = _clamp(1.5 * yaw_err, self.config.yaw_rate_limit)

        return cmd
