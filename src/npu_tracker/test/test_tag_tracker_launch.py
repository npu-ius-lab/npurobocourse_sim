import math
import os
import time
import unittest

os.environ.setdefault('ROS_DOMAIN_ID', '36')
os.environ.setdefault('GZ_PARTITION', 'npu_test_tag_tracker')

import launch
import launch_testing
import launch_testing.actions
import pytest
from launch.launch_description_sources import PythonLaunchDescriptionSource

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped, Twist


WAIT_SEC = 120.0
MARKER_XY = (2.0, 1.0)
LIMITS = {'linear.x': 0.8, 'linear.y': 0.8, 'linear.z': 0.5, 'angular.z': 0.8}


def _include(path, **kwargs):
    return launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=kwargs.items(),
    )


@pytest.mark.launch_test
def generate_test_description():
    bringup = get_package_share_directory('npu_sim_bringup')
    tracker = get_package_share_directory('npu_tracker')
    return launch.LaunchDescription([
        _include(os.path.join(bringup, 'launch', 'sim.launch.py'), headless='true'),
        _include(os.path.join(tracker, 'launch', 'tag_tracker.launch.py')),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TagTrackerProbe(Node):

    def __init__(self):
        super().__init__('tag_tracker_launch_probe')
        self.drone_pose = None
        self.commands = []
        self.diagnostics = []
        self.marker_active = False
        self._marker_pub = self.create_publisher(
            PoseStamped, '/npu/marker/pose', 10)
        self.create_subscription(
            PoseStamped, '/npu/drone/pose',
            lambda msg: setattr(self, 'drone_pose', msg), 10)
        self.create_subscription(
            Twist, '/npu/drone/cmd_vel',
            lambda msg: self.commands.append(msg), 10)
        self.create_subscription(
            DiagnosticArray, '/diagnostics',
            lambda msg: self.diagnostics.append(msg), 10)
        self.create_timer(0.1, self._publish_marker)

    def _publish_marker(self):
        if not self.marker_active:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = MARKER_XY[0]
        msg.pose.position.y = MARKER_XY[1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self._marker_pub.publish(msg)

    def spin_until(self, predicate, timeout=WAIT_SEC):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if predicate():
                return True
        return False

    def diagnostic_level(self, name):
        for array in reversed(self.diagnostics):
            for status in array.status:
                if status.name == name:
                    return status.level
        return None


class TestTagTrackerLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = TagTrackerProbe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()

    def test_tracker_starts_and_waits(self):
        ready = self.probe.spin_until(
            lambda: self.probe.drone_pose is not None and len(self.probe.commands) > 0,
            timeout=60.0)
        self.assertTrue(ready, 'no drone pose or tracker command output')
        recent = self.probe.commands[-5:]
        for cmd in recent:
            self.assertAlmostEqual(cmd.linear.x, 0.0, places=6)
            self.assertAlmostEqual(cmd.linear.z, 0.0, places=6)

    def test_selected_marker_produces_bounded_command(self):
        self.probe.commands.clear()
        self.probe.marker_active = True
        try:
            got = self.probe.spin_until(
                lambda: self.probe.drone_pose is not None and any(
                    math.hypot(c.linear.x, c.linear.y) > 0.05 or abs(c.linear.z) > 0.05
                    for c in self.probe.commands),
                timeout=60.0)
            self.assertTrue(got, 'tracker produced no tracking command for the marker')
            for cmd in self.probe.commands:
                self.assertLessEqual(abs(cmd.linear.x), LIMITS['linear.x'] + 1e-6)
                self.assertLessEqual(abs(cmd.linear.y), LIMITS['linear.y'] + 1e-6)
                self.assertLessEqual(abs(cmd.linear.z), LIMITS['linear.z'] + 1e-6)
                self.assertLessEqual(abs(cmd.angular.z), LIMITS['angular.z'] + 1e-6)
            p = self.probe.drone_pose.pose.position
            toward = (MARKER_XY[0] - p.x, MARKER_XY[1] - p.y)
            if math.hypot(*toward) > 0.15:
                cmd = next(c for c in self.probe.commands
                           if math.hypot(c.linear.x, c.linear.y) > 0.05)
                dot = cmd.linear.x * toward[0] + cmd.linear.y * toward[1]
                self.assertGreater(dot, 0.0, 'command does not point toward the marker')
        finally:
            self.probe.marker_active = False

    def test_stale_marker_zero_command_and_diagnostics(self):
        self.probe.commands.clear()
        time.sleep(2.0)
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and len(self.probe.commands) < 10:
            rclpy.spin_once(self.probe, timeout_sec=0.2)
        self.assertGreaterEqual(len(self.probe.commands), 10,
                                'no commands received after marker loss')
        for cmd in self.probe.commands[-10:]:
            self.assertAlmostEqual(cmd.linear.x, 0.0, places=6)
            self.assertAlmostEqual(cmd.linear.y, 0.0, places=6)
            self.assertAlmostEqual(cmd.linear.z, 0.0, places=6)
        seen = self.probe.spin_until(
            lambda: self.probe.diagnostic_level('tag_tracker/marker_pose')
            == DiagnosticStatus.ERROR,
            timeout=15.0)
        self.assertTrue(seen, 'no ERROR diagnostic for stale marker pose')


def teardown_module():
    time.sleep(10)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -6, -15, 130, 137],
        )
