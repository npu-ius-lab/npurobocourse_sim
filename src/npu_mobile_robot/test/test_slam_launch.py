import math
import os
import time
import unittest

os.environ.setdefault('ROS_DOMAIN_ID', '32')
os.environ.setdefault('GZ_PARTITION', 'npu_test_slam')

import launch
import launch_testing
import launch_testing.actions
import pytest
from launch.launch_description_sources import PythonLaunchDescriptionSource

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener


WAIT_SEC = 120.0


def _map_counts(msg):
    known = sum(1 for value in msg.data if value >= 0)
    occupied = sum(1 for value in msg.data if value > 50)
    return known, occupied


def _include(path, **kwargs):
    return launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=kwargs.items(),
    )


@pytest.mark.launch_test
def generate_test_description():
    bringup = get_package_share_directory('npu_sim_bringup')
    mobile = get_package_share_directory('npu_mobile_robot')
    return launch.LaunchDescription([
        _include(os.path.join(bringup, 'launch', 'sim.launch.py'), headless='true'),
        _include(os.path.join(mobile, 'launch', 'slam.launch.py')),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class SlamProbe(Node):

    def __init__(self):
        super().__init__('slam_launch_probe')
        self.map_msg = None
        self.odom = None
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.create_subscription(Odometry, '/npu/robot/odom', self._on_odom, 10)
        self._cmd_pub = self.create_publisher(Twist, '/npu/robot/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_map(self, msg):
        if msg.info.width > 0:
            self.map_msg = msg

    def _on_odom(self, msg):
        self.odom = msg

    def position(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        return (p.x, p.y)

    def spin_until(self, predicate, timeout=WAIT_SEC):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if predicate():
                return True
        return False


class TestSlamLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = SlamProbe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()

    def test_slam_lifecycle_active(self):
        client = self.probe.create_client(GetState, '/slam_toolbox/get_state')
        self.assertTrue(client.wait_for_service(timeout_sec=30.0))
        future = client.call_async(GetState.Request())
        self.assertTrue(self.probe.spin_until(lambda: future.done(), 30.0))
        self.assertEqual(future.result().current_state.label, 'active')

    def test_map_output(self):
        twist = Twist()
        twist.linear.x = 0.1
        for _ in range(30):
            self.probe._cmd_pub.publish(twist)
            rclpy.spin_once(self.probe, timeout_sec=0.05)
        found = self.probe.spin_until(lambda: self.probe.map_msg is not None)
        self.assertTrue(found, 'no /map message received')
        self.assertGreater(self.probe.map_msg.info.width, 0)
        occupied = [c for c in self.probe.map_msg.data if c > 50]
        self.assertGreater(len(occupied), 0, 'map contains no occupied cells')

    def test_in_place_rotation_updates_map(self):
        self.assertTrue(
            self.probe.spin_until(lambda: self.probe.map_msg is not None),
            'no initial /map message')
        self.assertTrue(
            self.probe.spin_until(lambda: self.probe.odom is not None),
            'no robot odometry')
        # 先等静止地图稳定，确保后续增长确由旋转引起。
        last_counts = None
        last_change = time.monotonic()
        settle_deadline = last_change + 15.0
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(self.probe, timeout_sec=0.2)
            counts = _map_counts(self.probe.map_msg)
            now = time.monotonic()
            if counts != last_counts:
                last_counts = counts
                last_change = now
            if now - last_change >= 3.0:
                break
        self.assertGreaterEqual(
            time.monotonic() - last_change, 3.0,
            'map did not stabilize before in-place rotation')
        before_known, before_occupied = _map_counts(self.probe.map_msg)
        start_position = self.probe.position()

        turn = Twist()
        turn.angular.z = 0.8
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            self.probe._cmd_pub.publish(turn)
            rclpy.spin_once(self.probe, timeout_sec=0.05)
        for _ in range(3):
            self.probe._cmd_pub.publish(Twist())
            rclpy.spin_once(self.probe, timeout_sec=0.05)

        def expanded():
            known, occupied = _map_counts(self.probe.map_msg)
            return known >= before_known and occupied >= before_occupied + 5

        self.assertTrue(
            self.probe.spin_until(expanded, 30.0),
            'in-place rotation did not add newly visible structure to the map')
        end_position = self.probe.position()
        self.assertLess(
            math.hypot(end_position[0] - start_position[0],
                       end_position[1] - start_position[1]),
            0.05,
            'map update test was not an in-place rotation',
        )

    def test_slam_tf_root(self):
        ok = self.probe.spin_until(lambda: self.probe.tf_buffer.can_transform(
            'world', 'npu_robot/odom', rclpy.time.Time()))
        self.assertTrue(ok, 'world -> npu_robot/odom TF not published by SLAM')


def teardown_module():
    time.sleep(10)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -6, -15, 130, 137],
        )
