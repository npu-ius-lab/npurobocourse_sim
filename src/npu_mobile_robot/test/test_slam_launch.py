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
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self._cmd_pub = self.create_publisher(Twist, '/npu/robot/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_map(self, msg):
        if self.map_msg is None and msg.info.width > 0:
            self.map_msg = msg

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
