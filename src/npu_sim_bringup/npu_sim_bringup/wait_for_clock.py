import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from rosgraph_msgs.msg import Clock


CLOCK_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class WaitForClock(Node):

    def __init__(self, timeout_sec: float):
        super().__init__('wait_for_clock')
        self._received = False
        self.create_subscription(Clock, '/clock', self._on_clock, CLOCK_QOS)
        self._deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout_sec
        self.create_timer(0.1, self._check)

    def _on_clock(self, _msg: Clock) -> None:
        self._received = True

    def _check(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now > self._deadline:
            self.get_logger().fatal('timed out waiting for /clock')
            raise SystemExit(2)


def main(args=None):
    timeout_sec = 30.0
    argv = sys.argv[1:] if args is None else args
    if '--timeout' in argv:
        timeout_sec = float(argv[argv.index('--timeout') + 1])

    rclpy.init(args=args)
    node = WaitForClock(timeout_sec)
    try:
        while rclpy.ok() and not node._received:
            rclpy.spin_once(node, timeout_sec=0.1)
    except SystemExit as exc:
        node.destroy_node()
        rclpy.shutdown()
        return exc.code
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
