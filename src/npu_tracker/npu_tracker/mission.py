import math

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

from npu_tracker.mission_logic import (
    FAILED,
    Mission,
    MissionConfig,
)


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MissionNode(Node):

    def __init__(self, **kwargs):
        super().__init__('mission', **kwargs)

        self.declare_parameter('takeoff_height', 1.0)
        self.declare_parameter('ring_approach', [-1.0, 2.8, 1.0])
        self.declare_parameter('ring_exit', [-1.0, 5.8, 1.0])
        self.declare_parameter('waypoints', [2.5, 2.0, 1.0])
        self.declare_parameter('position_tolerance', 0.25)
        self.declare_parameter('cruise_speed', 0.8)
        self.declare_parameter('vertical_speed', 0.5)
        self.declare_parameter('yaw_rate_limit', 0.8)
        self.declare_parameter('approach_distance_gain', 1.0)
        self.declare_parameter('state_timeout_sec', 120.0)
        self.declare_parameter('pose_timeout_sec', 1.0)

        flat_waypoints = list(self.get_parameter('waypoints').value)
        if not flat_waypoints or len(flat_waypoints) % 3 != 0:
            raise ValueError('waypoints must be a non-empty flat list of xyz triplets')
        waypoints = [tuple(flat_waypoints[i:i + 3]) for i in range(0, len(flat_waypoints), 3)]

        config = MissionConfig(
            takeoff_height=self.get_parameter('takeoff_height').value,
            ring_approach=tuple(self.get_parameter('ring_approach').value),
            ring_exit=tuple(self.get_parameter('ring_exit').value),
            waypoints=waypoints,
            position_tolerance=self.get_parameter('position_tolerance').value,
            cruise_speed=self.get_parameter('cruise_speed').value,
            vertical_speed=self.get_parameter('vertical_speed').value,
            yaw_rate_limit=self.get_parameter('yaw_rate_limit').value,
            approach_distance_gain=self.get_parameter('approach_distance_gain').value,
            state_timeout_sec=self.get_parameter('state_timeout_sec').value,
        )
        self._mission = Mission(config)
        self._pose_timeout = self.get_parameter('pose_timeout_sec').value

        self._last_pose = None
        self._last_pose_wall = None
        self._last_state = None

        self._cmd_pub = self.create_publisher(Twist, '/npu/drone/cmd_vel', 10)
        self._state_pub = self.create_publisher(String, '/npu/mission/state', 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.create_subscription(PoseStamped, '/npu/drone/pose', self._on_pose, 10)

        self.create_timer(0.05, self._tick)
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info('mission node started, state IDLE')

    def _wall_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != 'world':
            self.get_logger().warn(
                f'ignoring pose in frame {msg.header.frame_id!r}, expected world')
            return
        if not all(math.isfinite(v) for v in (
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w)):
            self.get_logger().warn('ignoring non-finite pose')
            return
        self._last_pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            _yaw_from_quaternion(msg.pose.orientation),
        )
        self._last_pose_wall = self._wall_now()

    def _fresh_pose(self):
        if self._last_pose is None or self._last_pose_wall is None:
            return None
        if self._wall_now() - self._last_pose_wall > self._pose_timeout:
            return None
        return self._last_pose

    def _tick(self) -> None:
        cmd, state = self._mission.update(self._fresh_pose(), self._wall_now())
        self._cmd_pub.publish(cmd)
        if state != self._last_state:
            self.get_logger().info(f'mission state: {state}')
            self._last_state = state
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)

    def _publish_diagnostics(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'mission/state'
        status.hardware_id = 'npu_drone'
        state = self._mission.state
        status.message = state
        if state == FAILED:
            status.level = DiagnosticStatus.ERROR
        elif self._fresh_pose() is None and state != 'IDLE':
            status.level = DiagnosticStatus.WARN
            status.message = f'{state} (pose stale, commanding zero)'
        else:
            status.level = DiagnosticStatus.OK
        array.status.append(status)
        self._diag_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
