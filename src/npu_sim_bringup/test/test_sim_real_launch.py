import math
import os
import re
import subprocess
import time
import unittest

os.environ.setdefault('ROS_DOMAIN_ID', '37')
os.environ.setdefault('GZ_PARTITION', 'npu_test_sim_real')

import launch
import launch_testing
import launch_testing.actions
import pytest
from launch.launch_description_sources import PythonLaunchDescriptionSource

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped


WAIT_SEC = 120.0
TARGET = (1.5, 0.5, 1.2)
ENTITY = 'npu_drone_display'


@pytest.mark.launch_test
def generate_test_description():
    bringup_share = get_package_share_directory('npu_sim_bringup')
    launch_file = os.path.join(bringup_share, 'launch', 'sim_real.launch.py')
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'headless': 'true'}.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ]), {}


def entity_position(name=ENTITY):
    out = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/world/course_world/pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=30)
    blocks = re.split(r'\n(?=pose \{)', out.stdout)
    for block in blocks:
        if f'name: "{name}"' not in block:
            continue
        match = re.search(r'position \{(.*?)\}', block, re.DOTALL)
        if match:
            values = dict(re.findall(r'([xyz]): ([-\d.e]+)', match.group(1)))
            return tuple(float(values.get(axis, '0.0')) for axis in 'xyz')
    return None


class DisplayProbe(Node):

    def __init__(self):
        super().__init__('sim_real_launch_probe')
        self.diagnostics = []
        self.pose_active = False
        self._pose_pub = self.create_publisher(
            PoseStamped, '/npu/external/drone_pose', 10)
        self.create_subscription(
            DiagnosticArray, '/diagnostics',
            lambda msg: self.diagnostics.append(msg), 10)
        self.create_timer(0.05, self._publish_pose)

    def _publish_pose(self):
        if not self.pose_active:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = TARGET[0]
        msg.pose.position.y = TARGET[1]
        msg.pose.position.z = TARGET[2]
        msg.pose.orientation.w = 1.0
        self._pose_pub.publish(msg)

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

    def wait_entity_position(self, predicate, timeout=WAIT_SEC):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                position = entity_position()
            except subprocess.TimeoutExpired:
                continue
            if position is not None and predicate(position):
                return position
            time.sleep(1.0)
        return None


class TestSimRealLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = DisplayProbe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()

    def test_display_entity_spawned(self):
        position = self.probe.wait_entity_position(lambda p: True, timeout=90.0)
        self.assertIsNotNone(position, f'entity {ENTITY} not found in world pose info')

    def test_valid_pose_moves_entity(self):
        self.probe.pose_active = True
        position = self.probe.wait_entity_position(
            lambda p: math.dist(p, TARGET) < 0.05, timeout=60.0)
        self.assertIsNotNone(
            position,
            f'entity did not move to target {TARGET}, last position: {position}')
        ok = self.probe.spin_until(
            lambda: self.probe.diagnostic_level('external_pose_display/display')
            == DiagnosticStatus.OK,
            timeout=15.0)
        self.assertTrue(ok, 'no OK diagnostic while following a valid pose')

    def test_stale_pose_stops_updates(self):
        self.probe.pose_active = True
        position = self.probe.wait_entity_position(
            lambda p: math.dist(p, TARGET) < 0.05, timeout=60.0)
        self.assertIsNotNone(
            position, f'entity did not reach target {TARGET} before stale check')
        self.probe.pose_active = False
        time.sleep(3.0)
        first = self.probe.wait_entity_position(lambda p: True, timeout=30.0)
        self.assertIsNotNone(first)
        time.sleep(2.0)
        second = entity_position()
        self.assertIsNotNone(second)
        self.assertLess(math.dist(first, second), 1e-3,
                        f'entity kept moving after pose went stale: {first} -> {second}')
        error = self.probe.spin_until(
            lambda: self.probe.diagnostic_level('external_pose_display/display')
            == DiagnosticStatus.ERROR,
            timeout=15.0)
        self.assertTrue(error, 'no ERROR diagnostic for stale external pose')
        self.assertLess(math.dist(second, TARGET), 0.05,
                        'entity drifted away from the last valid pose')


def teardown_module():
    time.sleep(10)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -6, -15, 130, 137],
        )
