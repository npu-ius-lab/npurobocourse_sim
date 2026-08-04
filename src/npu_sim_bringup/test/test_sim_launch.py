import os
import subprocess
import time
import unittest

os.environ.setdefault('ROS_DOMAIN_ID', '31')
os.environ.setdefault('GZ_PARTITION', 'npu_test_bringup')

import launch
import launch_testing
import launch_testing.actions
import pytest
from launch.launch_description_sources import PythonLaunchDescriptionSource

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


TOPIC_WAIT_SEC = 90.0


@pytest.mark.launch_test
def generate_test_description():
    bringup_share = get_package_share_directory('npu_sim_bringup')
    launch_file = os.path.join(bringup_share, 'launch', 'sim.launch.py')
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                'headless': 'true',
                'camera_preset': 'down',
            }.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class ProbeNode(Node):

    def __init__(self):
        super().__init__('sim_launch_probe')
        self.received = {}
        self._subs = []
        topics = {
            '/npu/drone/odom': (Odometry, 10),
            '/npu/drone/pose': (PoseStamped, 10),
            '/npu/robot/odom': (Odometry, 10),
            '/npu/robot/scan': (LaserScan, qos_profile_sensor_data),
            '/npu/drone/camera/image_raw': (Image, qos_profile_sensor_data),
            '/npu/drone/camera/camera_info': (CameraInfo, 10),
        }
        for name, (msg_type, qos) in topics.items():
            self._subs.append(self.create_subscription(
                msg_type, name, self._make_cb(name), qos))
        self._cmd_pub = self.create_publisher(Twist, '/npu/drone/cmd_vel', 10)
        self._cmd_gz_sub = self.create_subscription(
            Twist, '/npu/drone/cmd_vel_gz', self._make_cb('/npu/drone/cmd_vel_gz'), 10)
        self._enable_sub = self.create_subscription(
            Bool, '/npu/drone/enable', self._make_cb('/npu/drone/enable'), 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _make_cb(self, name):
        def cb(msg):
            self.received[name] = msg
        return cb

    def wait_for(self, topic, timeout=TOPIC_WAIT_SEC):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if topic in self.received:
                return self.received[topic]
        raise AssertionError(f'timed out waiting for message on {topic}')


class TestSimLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = ProbeNode()

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()

    def test_clock_running(self):
        names = [t for t, _ in self.probe.get_topic_names_and_types()]
        deadline = time.monotonic() + TOPIC_WAIT_SEC
        while '/clock' not in names and time.monotonic() < deadline:
            time.sleep(0.5)
            names = [t for t, _ in self.probe.get_topic_names_and_types()]
        self.assertIn('/clock', names)

    def test_drone_odom_in_world(self):
        msg = self.probe.wait_for('/npu/drone/odom')
        self.assertEqual(msg.header.frame_id, 'world')
        self.assertEqual(msg.child_frame_id, 'npu_drone/base_link')

    def test_drone_pose_relay(self):
        msg = self.probe.wait_for('/npu/drone/pose')
        self.assertEqual(msg.header.frame_id, 'world')

    def test_drone_enable_published(self):
        msg = self.probe.wait_for('/npu/drone/enable')
        self.assertTrue(msg.data)

    def test_drone_command_validation_path(self):
        self.probe.wait_for('/npu/drone/odom')
        cmd = Twist()
        cmd.linear.x = 0.5
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            self.probe._cmd_pub.publish(cmd)
            rclpy.spin_once(self.probe, timeout_sec=0.2)
            msg = self.probe.received.get('/npu/drone/cmd_vel_gz')
            if msg is not None and abs(msg.linear.x - 0.5) < 1e-6:
                return
        self.fail('validated command 0.5 not observed on /npu/drone/cmd_vel_gz')

    def test_robot_interfaces(self):
        odom = self.probe.wait_for('/npu/robot/odom')
        self.assertEqual(odom.header.frame_id, 'npu_robot/odom')
        scan = self.probe.wait_for('/npu/robot/scan')
        self.assertEqual(scan.header.frame_id, 'npu_robot/lidar_link')
        self.assertGreater(len(scan.ranges), 0)

    def test_camera_topics(self):
        info = self.probe.wait_for('/npu/drone/camera/camera_info')
        self.assertGreater(info.width, 0)
        image = self.probe.wait_for('/npu/drone/camera/image_raw')
        self.assertEqual(image.width, info.width)

    def test_robot_tf_root(self):
        deadline = time.monotonic() + TOPIC_WAIT_SEC
        while time.monotonic() < deadline:
            rclpy.spin_once(self.probe, timeout_sec=0.2)
            if self.probe.tf_buffer.can_transform(
                    'npu_robot/odom', 'npu_robot/base_link', rclpy.time.Time()):
                return
        self.fail('npu_robot/odom -> npu_robot/base_link TF not available')

    def test_entities_spawned(self):
        deadline = time.monotonic() + TOPIC_WAIT_SEC
        while time.monotonic() < deadline:
            try:
                out = subprocess.run(
                    ['gz', 'topic', '-e', '-t', '/world/course_world/pose/info', '-n', '1'],
                    capture_output=True, text=True, timeout=30)
                text = out.stdout
                if 'npu_drone' in text and 'npu_robot' in text and 'ring' in text:
                    return
            except subprocess.TimeoutExpired:
                pass
            time.sleep(2.0)
        self.fail('expected entities npu_drone/npu_robot/ring not found in world pose info')


def teardown_module():
    time.sleep(10)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -6, -15, 130, 137],
        )
