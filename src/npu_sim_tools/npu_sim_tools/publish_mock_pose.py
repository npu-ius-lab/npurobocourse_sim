import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from npu_sim_tools.mock_trajectory import MODES, TrajectoryConfig, sample


class MockPosePublisher(Node):
    """Publishes a repeatable mock external localization trajectory on
    /npu/external/drone_pose, replacing a real localization mat or drone
    during virtual-real display acceptance."""

    def __init__(self):
        super().__init__('publish_mock_pose')

        self.declare_parameter('mode', 'loop')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('start_pose', [0.0, 0.0, 1.0])
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('speed', 0.3)
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('seed', 0)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('topic', '/npu/external/drone_pose')

        start = list(self.get_parameter('start_pose').value)
        if len(start) != 3:
            raise ValueError('start_pose must be [x, y, z]')
        self._config = TrajectoryConfig(
            mode=self.get_parameter('mode').value,
            start=(float(start[0]), float(start[1]), float(start[2])),
            yaw=float(self.get_parameter('start_yaw').value),
            speed=float(self.get_parameter('speed').value),
            radius=float(self.get_parameter('radius').value),
            seed=int(self.get_parameter('seed').value),
        )
        if self._config.mode not in MODES:
            raise ValueError(
                f"unknown mode {self._config.mode!r}, expected one of {MODES}")
        if self._config.radius <= 0.0:
            raise ValueError('radius must be positive')

        self._frame_id = self.get_parameter('frame_id').value
        rate_hz = float(self.get_parameter('rate_hz').value)
        if rate_hz <= 0.0:
            raise ValueError('rate_hz must be positive')

        self._pub = self.create_publisher(
            PoseStamped, self.get_parameter('topic').value, 10)
        self._start_time = self.get_clock().now()
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f'publish_mock_pose started, mode={self._config.mode} '
            f'rate={rate_hz}Hz start={self._config.start}')

    def _tick(self) -> None:
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        x, y, z, yaw = sample(self._config, elapsed)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockPosePublisher()
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
