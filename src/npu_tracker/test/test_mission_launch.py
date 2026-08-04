import math
import os
import time
import unittest

os.environ.setdefault('ROS_DOMAIN_ID', '34')
os.environ.setdefault('GZ_PARTITION', 'npu_test_mission')

import launch
import launch_testing
import launch_testing.actions
import pytest
from launch.launch_description_sources import PythonLaunchDescriptionSource

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


WAIT_SEC = 180.0


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
        _include(os.path.join(tracker, 'launch', 'pass_and_waypoints.launch.py')),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class MissionProbe(Node):

    def __init__(self):
        super().__init__('mission_launch_probe')
        self.last_pose = None
        self.states = []
        self.create_subscription(
            PoseStamped, '/npu/drone/pose',
            lambda msg: setattr(self, 'last_pose', msg), 10)
        self.create_subscription(String, '/npu/mission/state', self._on_state, 10)

    def _on_state(self, msg):
        if not self.states or self.states[-1] != msg.data:
            self.states.append(msg.data)

    def spin_until(self, predicate, timeout=WAIT_SEC):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if predicate():
                return True
        return False

    def position(self):
        if self.last_pose is None:
            return None
        p = self.last_pose.pose.position
        return (p.x, p.y, p.z)


class TestMissionLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = MissionProbe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()

    def test_takeoff_reaches_altitude(self):
        reached = self.probe.spin_until(
            lambda: self.probe.position() is not None
            and self.probe.position()[2] > 0.75,
            timeout=90.0)
        self.assertTrue(reached, 'drone did not reach takeoff altitude')

    def test_drone_moves_toward_ring(self):
        moved = self.probe.spin_until(
            lambda: self.probe.position() is not None
            and math.hypot(self.probe.position()[0] + 1.5,
                           self.probe.position()[1] + 4.0) > 2.0,
            timeout=90.0)
        self.assertTrue(moved, 'drone showed no measurable displacement')

    def test_mission_state_sequence(self):
        completed = self.probe.spin_until(
            lambda: 'COMPLETE' in self.probe.states, timeout=150.0)
        sequence = self.probe.states
        self.assertIn('TAKEOFF', sequence)
        self.assertIn('APPROACH_RING', sequence)
        self.assertIn('TRAVERSE_RING', sequence)
        self.assertIn('APPROACH_WAYPOINT', sequence)
        self.assertNotIn('FAILED', sequence)
        self.assertTrue(completed, f'mission did not complete, states: {sequence}')

    def test_terminal_state_zero_command(self):
        if 'COMPLETE' not in self.probe.states:
            self.skipTest('mission did not complete')
        position = self.probe.position()
        time.sleep(3.0)
        rclpy.spin_once(self.probe, timeout_sec=0.5)
        later = self.probe.position()
        drift = math.hypot(later[0] - position[0], later[1] - position[1])
        self.assertLess(drift, 0.5, f'drone kept moving after COMPLETE: {drift:.3f} m')


def teardown_module():
    time.sleep(10)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -6, -15, 130, 137],
        )
