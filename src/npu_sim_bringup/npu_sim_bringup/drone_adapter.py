import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from npu_sim_bringup.command_gate import CommandGate, CommandLimits


class DroneAdapter(Node):

    def __init__(self):
        super().__init__('drone_adapter')

        self.declare_parameter('max_linear_xy', 2.0)
        self.declare_parameter('max_vertical', 1.0)
        self.declare_parameter('max_yaw_rate', 1.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('odom_timeout_sec', 1.0)
        self.declare_parameter('world_frame', 'world')

        limits = CommandLimits(
            max_linear_xy=self.get_parameter('max_linear_xy').value,
            max_vertical=self.get_parameter('max_vertical').value,
            max_yaw_rate=self.get_parameter('max_yaw_rate').value,
        )
        self._gate = CommandGate(
            limits=limits,
            command_timeout_sec=self.get_parameter('command_timeout_sec').value,
        )
        self._odom_timeout = self.get_parameter('odom_timeout_sec').value
        self._world_frame = self.get_parameter('world_frame').value

        self._last_odom_stamp = None
        self._odom_frame_ok = True
        self._last_error = ''

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._cmd_pub = self.create_publisher(Twist, '/npu/drone/cmd_vel_gz', 10)
        self._enable_pub = self.create_publisher(Bool, '/npu/drone/enable', 10)
        self._pose_pub = self.create_publisher(PoseStamped, '/npu/drone/pose', 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.create_subscription(Twist, '/npu/drone/cmd_vel', self._on_command, 10)
        self.create_subscription(Odometry, '/npu/drone/odom', self._on_odom, sensor_qos)

        self.create_timer(0.05, self._watchdog)
        self.create_timer(1.0, self._publish_enable)
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info('drone_adapter started')

    def _wall_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_command(self, msg: Twist) -> None:
        if not self._odom_fresh():
            self._last_error = 'odometry unavailable, command held'
            return
        clamped, error = self._gate.validate(msg, self._wall_now())
        if error is not None:
            self._last_error = error
            self.get_logger().warn(error)
            return
        self._cmd_pub.publish(clamped)

    def _on_odom(self, msg: Odometry) -> None:
        if msg.header.frame_id != self._world_frame:
            self._odom_frame_ok = False
            self._last_error = f'odometry frame {msg.header.frame_id!r} != {self._world_frame!r}'
            return
        self._odom_frame_ok = True
        self._last_odom_stamp = self._wall_now()
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._pose_pub.publish(pose)

    def _odom_fresh(self) -> bool:
        if self._last_odom_stamp is None:
            return False
        return (self._wall_now() - self._last_odom_stamp) <= self._odom_timeout

    def _watchdog(self) -> None:
        zero, timed_out = self._gate.watchdog(self._wall_now())
        if zero is not None:
            self._cmd_pub.publish(zero)
        if timed_out and not self._odom_fresh():
            self._gate.reset()

    def _publish_enable(self) -> None:
        msg = Bool()
        msg.data = True
        self._enable_pub.publish(msg)

    def _status(self, name: str, level: int, message: str) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = f'drone_adapter/{name}'
        status.level = level
        status.message = message
        status.hardware_id = 'npu_drone'
        if self._last_error:
            status.values.append(KeyValue(key='last_error', value=self._last_error))
        return status

    def _publish_diagnostics(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        if self._odom_fresh() and self._odom_frame_ok:
            array.status.append(self._status('bridge', DiagnosticStatus.OK, 'odometry streaming'))
        else:
            array.status.append(self._status(
                'bridge', DiagnosticStatus.ERROR,
                'odometry stale or missing, adapter inactive'))
        if self._gate.is_timed_out(self._wall_now()):
            array.status.append(self._status(
                'command', DiagnosticStatus.WARN, 'no fresh command, publishing zero'))
        else:
            array.status.append(self._status('command', DiagnosticStatus.OK, 'command active'))
        self._diag_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = DroneAdapter()
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
