import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from npu_sim_bringup.command_gate import CommandGate


class RobotAdapter(Node):

    def __init__(self):
        super().__init__('robot_adapter')
        self.declare_parameter('command_timeout_sec', 0.5)
        self._gate = CommandGate(
            command_timeout_sec=self.get_parameter('command_timeout_sec').value)
        self._cmd_pub = self.create_publisher(Twist, '/npu/robot/cmd_vel_gz', 10)
        self.create_subscription(
            Twist, '/npu/robot/cmd_vel', self._on_command, 10)
        self.create_timer(0.05, self._watchdog)

    def _wall_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_command(self, msg: Twist) -> None:
        self._gate.mark_received(self._wall_now())
        self._cmd_pub.publish(msg)

    def _watchdog(self) -> None:
        zero, _ = self._gate.watchdog(self._wall_now())
        if zero is not None:
            self._cmd_pub.publish(zero)


def main(args=None):
    rclpy.init(args=args)
    node = RobotAdapter()
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
