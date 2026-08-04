import os

os.environ.setdefault('ROS_DOMAIN_ID', '36')

import rclpy

from geometry_msgs.msg import Twist

from npu_sim_bringup.command_gate import CommandGate
from npu_sim_bringup.drone_adapter import DroneAdapter


class _FakePub:

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _DroneAdapterFixture:

    def __init__(self):
        self.node = DroneAdapter()
        self.now = [100.0]
        self.node._wall_now = lambda: self.now[0]

    def destroy(self):
        self.node.destroy_node()
        rclpy.shutdown()


def test_command_held_while_odom_stale_does_not_arm_gate():
    rclpy.init()
    fixture = _DroneAdapterFixture()
    try:
        cmd = Twist()
        cmd.linear.x = 0.5
        fixture.node._on_command(cmd)
        assert fixture.node._gate._last_command_stamp is None
        assert fixture.node._gate.is_timed_out(fixture.now[0])
    finally:
        fixture.destroy()


def test_watchdog_publishes_zero_once_on_odom_loss():
    rclpy.init()
    fixture = _DroneAdapterFixture()
    try:
        fake = _FakePub()
        fixture.node._cmd_pub = fake
        fixture.node._gate = CommandGate()
        fixture.node._gate.validate(Twist(), fixture.now[0] - 1.0)
        fixture.node._watchdog()
        fixture.node._watchdog()
        assert len(fake.messages) == 1
        assert fake.messages[0].linear.x == 0.0
        assert fake.messages[0].linear.y == 0.0
        assert fake.messages[0].linear.z == 0.0
    finally:
        fixture.destroy()


def test_fresh_command_forwarded_when_odom_fresh():
    rclpy.init()
    fixture = _DroneAdapterFixture()
    try:
        fake = _FakePub()
        fixture.node._cmd_pub = fake
        fixture.node._last_odom_stamp = fixture.now[0] - 0.2
        cmd = Twist()
        cmd.linear.x = 0.5
        fixture.node._on_command(cmd)
        assert len(fake.messages) == 1
        assert fake.messages[0].linear.x == 0.5
        assert fixture.node._gate._last_command_stamp == fixture.now[0]
    finally:
        fixture.destroy()
