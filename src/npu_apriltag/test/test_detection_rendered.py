import math
import os
import struct
import time
import unittest
import zlib

os.environ.setdefault('ROS_DOMAIN_ID', '37')

import launch
import launch_testing
import launch_testing.actions
import pytest
from launch.launch_description_sources import PythonLaunchDescriptionSource

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image


def _read_png_grayscale(path):
    with open(path, 'rb') as f:
        data = f.read()
    pos = 8
    idat = b''
    width = height = None
    while pos < len(data):
        length = struct.unpack('>I', data[pos:pos + 4])[0]
        ctype = data[pos + 4:pos + 8]
        chunk = data[pos + 8:pos + 8 + length]
        if ctype == b'IHDR':
            width, height, bitdepth, colortype = struct.unpack('>IIBB', chunk[:10])
            assert bitdepth == 8 and colortype == 0, 'expected 8-bit grayscale PNG'
        elif ctype == b'IDAT':
            idat += chunk
        pos += 12 + length
    raw = zlib.decompress(idat)
    stride = width + 1
    img = bytearray()
    prev = bytearray(width)
    for y in range(height):
        line = raw[y * stride:(y + 1) * stride]
        f0 = line[0]
        cur = bytearray(line[1:])
        for x in range(width):
            a = cur[x - 1] if x > 0 else 0
            b = prev[x]
            c = prev[x - 1] if x > 0 else 0
            if f0 == 1:
                cur[x] = (cur[x] + a) & 0xFF
            elif f0 == 2:
                cur[x] = (cur[x] + b) & 0xFF
            elif f0 == 3:
                cur[x] = (cur[x] + (a + b) // 2) & 0xFF
            elif f0 == 4:
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                cur[x] = (cur[x] + pr) & 0xFF
        img += cur
        prev = cur
    return width, height, bytes(img)


@pytest.mark.launch_test
def generate_test_description():
    apriltag_share = get_package_share_directory('npu_apriltag')
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(apriltag_share, 'launch', 'detection.launch.py')),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class RenderedCameraProbe(Node):

    def __init__(self, width, height, pixels):
        super().__init__('rendered_camera_probe')
        self.width = width
        self.height = height
        self.pixels = pixels
        self.marker_poses = []
        self._image_pub = self.create_publisher(
            Image, '/npu/drone/camera/image_raw', 10)
        self._info_pub = self.create_publisher(
            CameraInfo, '/npu/drone/camera/camera_info', 10)
        self._drone_pub = self.create_publisher(
            PoseStamped, '/npu/drone/pose', 10)
        self.create_subscription(
            PoseStamped, '/npu/marker/pose',
            lambda msg: self.marker_poses.append(msg), 10)
        self.create_timer(0.1, self._publish)

    def _publish(self):
        stamp = self.get_clock().now().to_msg()

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = 'npu_drone/front_cam'
        image.height = self.height
        image.width = self.width
        image.encoding = 'mono8'
        image.step = self.width
        image.data = self.pixels
        self._image_pub.publish(image)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = 'npu_drone/front_cam'
        info.height = self.height
        info.width = self.width
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [500.0, 0.0, 256.0, 0.0, 500.0, 256.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [500.0, 0.0, 256.0, 0.0, 0.0, 500.0, 256.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._info_pub.publish(info)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'world'
        pose.pose.orientation.w = 1.0
        self._drone_pub.publish(pose)

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if predicate():
                return True
        return False


class TestRenderedCameraDetection(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        description_share = get_package_share_directory('npu_sim_description')
        tag_png = os.path.join(
            description_share, 'models', 'marker', 'meshes', 'tag36_11_00008.png')
        width, height, pixels = _read_png_grayscale(tag_png)
        cls.probe = RenderedCameraProbe(width, height, pixels)

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()

    def test_marker_pose_from_rendered_tag(self):
        received = self.probe.spin_until(
            lambda: len(self.probe.marker_poses) >= 10, timeout=60.0)
        self.assertTrue(received, 'no /npu/marker/pose from rendered tag image')

        pose = self.probe.marker_poses[-1]
        self.assertEqual(pose.header.frame_id, 'world')
        p = pose.pose.position
        # Tag face fills ~410 px of the 512 px image at fx=500, size 0.08 m,
        # so the marker sits ~0.098 m along the optical axis. With the drone
        # at the world origin and the forward 15-degree mount at
        # (0.05, 0, -0.25), the world marker pose is deterministic.
        self.assertAlmostEqual(p.x, 0.1446, delta=0.03)
        self.assertAlmostEqual(p.y, 0.0, delta=0.02)
        self.assertAlmostEqual(p.z, -0.2753, delta=0.03)

        # Deterministic detector input: successive estimates agree closely.
        xs = [m.pose.position.x for m in self.probe.marker_poses[-10:]]
        self.assertLess(max(xs) - min(xs), 0.01)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -6, -15, 130, 137],
        )
