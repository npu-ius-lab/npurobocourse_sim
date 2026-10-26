import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

from geometry_msgs.msg import Twist


@dataclass
class CommandLimits:
    max_linear_xy: float = 2.0
    max_vertical: float = 1.0
    max_yaw_rate: float = 1.0


@dataclass
class CommandGate:
    limits: CommandLimits = field(default_factory=CommandLimits)
    command_timeout_sec: float = 0.5
    _last_command_stamp: Optional[float] = None
    _timed_out: bool = True

    @staticmethod
    def _finite(twist: Twist) -> bool:
        values = (
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        )
        return all(math.isfinite(v) for v in values)

    def mark_received(self, now: float) -> None:
        # 超时只归零一次，下一条命令负责重新武装门控。
        self._last_command_stamp = now
        self._timed_out = False

    def validate(self, twist: Twist, now: float) -> Tuple[Optional[Twist], Optional[str]]:
        if not self._finite(twist):
            return None, 'rejected non-finite command'
        clamped = Twist()
        lim = self.limits
        clamped.linear.x = max(-lim.max_linear_xy, min(lim.max_linear_xy, twist.linear.x))
        clamped.linear.y = max(-lim.max_linear_xy, min(lim.max_linear_xy, twist.linear.y))
        clamped.linear.z = max(-lim.max_vertical, min(lim.max_vertical, twist.linear.z))
        clamped.angular.x = max(-lim.max_yaw_rate, min(lim.max_yaw_rate, twist.angular.x))
        clamped.angular.y = max(-lim.max_yaw_rate, min(lim.max_yaw_rate, twist.angular.y))
        clamped.angular.z = max(-lim.max_yaw_rate, min(lim.max_yaw_rate, twist.angular.z))
        self.mark_received(now)
        return clamped, None

    def is_timed_out(self, now: float) -> bool:
        if self._timed_out:
            return True
        if self._last_command_stamp is None:
            return True
        return now - self._last_command_stamp > self.command_timeout_sec

    def watchdog(self, now: float) -> Tuple[Optional[Twist], bool]:
        if self._timed_out:
            return None, True
        if self.is_timed_out(now):
            self._timed_out = True
            return Twist(), True
        return None, False

    def reset(self) -> None:
        self._last_command_stamp = None
        self._timed_out = True
