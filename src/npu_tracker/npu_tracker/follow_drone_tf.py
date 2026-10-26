import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped tf2 support

from npu_tracker.follow_logic import DirectFollower, FollowConfig
from npu_tracker.mission_logic import _clamp


class FollowDroneTfNode(Node):

    def __init__(self):
        super().__init__('follow_drone_tf')

        # Simulation-only workspace: default to sim time so bare `ros2 run`
        # matches the launch-file behavior (README interface convention).
        # rclpy's TimeSource already declared use_sim_time (default False);
        # an explicit override (launch file, tests) always wins.
        if 'use_sim_time' not in getattr(self, '_parameter_overrides', {}):
            self.set_parameters(
                [Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.declare_parameter('follow_distance', 1.5)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('linear_gain', 0.8)
        self.declare_parameter('angular_gain', 1.5)
        self.declare_parameter('pose_timeout_sec', 1.0)
        self.declare_parameter('tf_timeout_sec', 1.0)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('odom_frame', 'npu_robot/odom')
        self.declare_parameter('base_frame', 'npu_robot/base_link')
        self.declare_parameter('world_to_odom', [2.5, -1.5, 0.0])

        config = FollowConfig(
            follow_distance=self.get_parameter('follow_distance').value,
            goal_tolerance=self.get_parameter('goal_tolerance').value,
            max_linear=self.get_parameter('max_linear').value,
            max_angular=self.get_parameter('max_angular').value,
            linear_gain=self.get_parameter('linear_gain').value,
            angular_gain=self.get_parameter('angular_gain').value,
        )
        self._follower = DirectFollower(config)
        self._pose_timeout = self.get_parameter('pose_timeout_sec').value
        self._tf_timeout = self.get_parameter('tf_timeout_sec').value
        self._world_frame = self.get_parameter('world_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        world_to_odom = list(self.get_parameter('world_to_odom').value)
        if len(world_to_odom) != 3:
            raise ValueError('world_to_odom must be [x, y, yaw]')

        self._last_pose: PoseStamped = None
        self._last_pose_wall = None
        self._tf_error = ''

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_static = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_world_to_odom(world_to_odom)

        self._cmd_pub = self.create_publisher(Twist, '/npu/robot/cmd_vel', 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.create_subscription(PoseStamped, '/npu/drone/pose', self._on_pose, 10)

        self.create_timer(0.05, self._tick)
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            f'follow_drone_tf started, follow_distance={config.follow_distance}')

    def _wall_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_world_to_odom(self, world_to_odom) -> None:
        x, y, yaw = world_to_odom
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._world_frame
        transform.child_frame_id = self._odom_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self._tf_static.sendTransform(transform)

    def _on_pose(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != self._world_frame:
            self.get_logger().warn(
                f'ignoring pose in frame {msg.header.frame_id!r}, '
                f'expected {self._world_frame!r}')
            return
        self._last_pose = msg
        self._last_pose_wall = self._wall_now()

    def _fresh_pose(self):
        if self._last_pose is None or self._last_pose_wall is None:
            return None
        if self._wall_now() - self._last_pose_wall > self._pose_timeout:
            return None
        return self._last_pose

    def _drone_in_base_frame(self, pose: PoseStamped):
        try:
            robot_tf = self._tf_buffer.lookup_transform(
                self._odom_frame, self._base_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self._tf_error = f'robot TF unavailable: {exc}'
            return None
        stamp = Time.from_msg(robot_tf.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if age > self._tf_timeout:
            self._tf_error = f'robot TF stale ({age:.1f}s old)'
            return None
        self._tf_error = ''
        try:
            return self._tf_buffer.transform(pose, self._base_frame)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self._tf_error = f'drone pose transform failed: {exc}'
            return None

    def _tick(self) -> None:
        rel = None
        pose = self._fresh_pose()
        if pose is not None:
            drone_in_base = self._drone_in_base_frame(pose)
            if drone_in_base is not None:
                rel = (
                    drone_in_base.pose.position.x,
                    drone_in_base.pose.position.y,
                )
        self._cmd_pub.publish(self._follower.update(rel))

    def _publish_diagnostics(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        pose_status = DiagnosticStatus()
        pose_status.name = 'follow_drone_tf/drone_pose'
        pose_status.hardware_id = 'npu_robot'
        if self._fresh_pose() is not None:
            pose_status.level = DiagnosticStatus.OK
            pose_status.message = 'drone pose fresh'
        else:
            pose_status.level = DiagnosticStatus.ERROR
            pose_status.message = 'drone pose missing or stale, commanding zero'
        array.status.append(pose_status)

        tf_status = DiagnosticStatus()
        tf_status.name = 'follow_drone_tf/robot_tf'
        tf_status.hardware_id = 'npu_robot'
        if self._tf_error:
            tf_status.level = DiagnosticStatus.ERROR
            tf_status.message = f'{self._tf_error}, commanding zero'
        else:
            tf_status.level = DiagnosticStatus.OK
            tf_status.message = 'robot TF fresh'
        array.status.append(tf_status)

        self._diag_pub.publish(array)


def main(args=None):
    # 保留有效的退出 context，最终零命令才能发布并等待订阅确认。
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = FollowDroneTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._cmd_pub.publish(Twist())
        except Exception as exc:
            node.get_logger().error(f'failed to publish final zero command: {exc}')
        try:
            acknowledged = node._cmd_pub.wait_for_all_acked(
                Duration(seconds=0.2))
            if not acknowledged:
                node.get_logger().error(
                    'final zero command was not acknowledged within 0.2 seconds')
        except Exception as exc:
            node.get_logger().error(
                f'failed to acknowledge final zero command: {exc}')
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
