import math

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped, Twist

from npu_tracker.tag_tracker_logic import TagTracker, TagTrackerConfig


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TagTrackerNode(Node):
    """Consumes /npu/marker/pose and /npu/drone/pose and hovers the drone
    above the selected marker. Publishes a zero command whenever the marker
    pose or the drone pose is missing or stale."""

    def __init__(self):
        super().__init__('tag_tracker')

        self.declare_parameter('hover_height', 1.2)
        self.declare_parameter('position_tolerance', 0.15)
        self.declare_parameter('max_linear_xy', 0.8)
        self.declare_parameter('max_vertical', 0.5)
        self.declare_parameter('yaw_rate_limit', 0.8)
        self.declare_parameter('distance_gain', 1.0)
        self.declare_parameter('face_marker_distance', 0.5)
        self.declare_parameter('pose_timeout_sec', 1.0)
        self.declare_parameter('marker_topic', '/npu/marker/pose')
        self.declare_parameter('drone_pose_topic', '/npu/drone/pose')
        self.declare_parameter('world_frame', 'world')

        config = TagTrackerConfig(
            hover_height=self.get_parameter('hover_height').value,
            position_tolerance=self.get_parameter('position_tolerance').value,
            max_linear_xy=self.get_parameter('max_linear_xy').value,
            max_vertical=self.get_parameter('max_vertical').value,
            yaw_rate_limit=self.get_parameter('yaw_rate_limit').value,
            distance_gain=self.get_parameter('distance_gain').value,
            face_marker_distance=self.get_parameter('face_marker_distance').value,
        )
        self._tracker = TagTracker(config)
        self._pose_timeout = self.get_parameter('pose_timeout_sec').value
        self._world_frame = self.get_parameter('world_frame').value

        self._last_marker = None
        self._last_marker_wall = None
        self._last_pose = None
        self._last_pose_wall = None

        self._cmd_pub = self.create_publisher(Twist, '/npu/drone/cmd_vel', 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.create_subscription(
            PoseStamped,
            self.get_parameter('marker_topic').value,
            self._on_marker, 10)
        self.create_subscription(
            PoseStamped,
            self.get_parameter('drone_pose_topic').value,
            self._on_pose, 10)

        self.create_timer(0.05, self._tick)
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            f'tag_tracker started, hover_height={config.hover_height}')

    def _wall_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_marker(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != self._world_frame:
            self.get_logger().warn(
                f'ignoring marker pose in frame {msg.header.frame_id!r}, '
                f'expected {self._world_frame!r}')
            return
        self._last_marker = (
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self._last_marker_wall = self._wall_now()

    def _on_pose(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != self._world_frame:
            self.get_logger().warn(
                f'ignoring drone pose in frame {msg.header.frame_id!r}, '
                f'expected {self._world_frame!r}')
            return
        if not all(math.isfinite(v) for v in (
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w)):
            self.get_logger().warn('ignoring non-finite drone pose')
            return
        self._last_pose = (
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            _yaw_from_quaternion(msg.pose.orientation))
        self._last_pose_wall = self._wall_now()

    def _fresh(self, value, wall_stamp):
        if value is None or wall_stamp is None:
            return None
        if self._wall_now() - wall_stamp > self._pose_timeout:
            return None
        return value

    def _tick(self) -> None:
        marker = self._fresh(self._last_marker, self._last_marker_wall)
        pose = self._fresh(self._last_pose, self._last_pose_wall)
        self._cmd_pub.publish(self._tracker.update(marker, pose))

    def _publish_diagnostics(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        marker_status = DiagnosticStatus()
        marker_status.name = 'tag_tracker/marker_pose'
        marker_status.hardware_id = 'npu_drone'
        if self._fresh(self._last_marker, self._last_marker_wall) is not None:
            marker_status.level = DiagnosticStatus.OK
            marker_status.message = 'marker pose fresh'
        else:
            marker_status.level = DiagnosticStatus.ERROR
            marker_status.message = 'marker pose missing or stale, commanding zero'
        array.status.append(marker_status)

        pose_status = DiagnosticStatus()
        pose_status.name = 'tag_tracker/drone_pose'
        pose_status.hardware_id = 'npu_drone'
        if self._fresh(self._last_pose, self._last_pose_wall) is not None:
            pose_status.level = DiagnosticStatus.OK
            pose_status.message = 'drone pose fresh'
        else:
            pose_status.level = DiagnosticStatus.ERROR
            pose_status.message = 'drone pose missing or stale, commanding zero'
        array.status.append(pose_status)

        self._diag_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = TagTrackerNode()
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
