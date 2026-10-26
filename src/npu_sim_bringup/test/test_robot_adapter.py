import os

os.environ.setdefault('ROS_DOMAIN_ID', '38')

import rclpy

from geometry_msgs.msg import Twist

from npu_sim_bringup.robot_adapter import RobotAdapter


class _FakePub:

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def test_command_forwarding_timeout_and_recovery():
    rclpy.init()
    node = RobotAdapter()
    now = [100.0]
    node._wall_now = lambda: now[0]
    fake = _FakePub()
    node._cmd_pub = fake
    try:
        command = Twist()
        command.linear.x = 0.4
        node._on_command(command)
        assert len(fake.messages) == 1
        assert fake.messages[-1].linear.x == 0.4

        now[0] += 1.0
        node._watchdog()
        node._watchdog()
        assert len(fake.messages) == 2
        assert fake.messages[-1].linear.x == 0.0

        command.linear.x = -0.2
        node._on_command(command)
        assert len(fake.messages) == 3
        assert fake.messages[-1].linear.x == -0.2
        assert not node._gate.is_timed_out(now[0])
    finally:
        node.destroy_node()
        rclpy.shutdown()
