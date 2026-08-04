import os

os.environ.setdefault('ROS_DOMAIN_ID', '37')

import rclpy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo

from npu_apriltag.detection_adapter import DetectionAdapter


class _DetectionAdapterFixture:

    def __init__(self):
        self.node = DetectionAdapter()
        self.now = [100.0]
        self.node._wall_now = lambda: self.now[0]
        self.node.get_publishers_info_by_topic = lambda topic: [object()]
        info = CameraInfo()
        info.header.frame_id = self.node._camera_frame
        self.node._last_camera_info = info
        self.node._last_camera_info_wall = self.now[0]

    def destroy(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def send_drone_pose(self, at=None):
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        self.node._on_drone_pose(pose)
        if at is not None:
            self.node._last_drone_pose_wall = at


def test_stale_drone_pose_reported():
    rclpy.init()
    fixture = _DetectionAdapterFixture()
    try:
        fixture.send_drone_pose(at=fixture.now[0] - 5.0)
        assert fixture.node._check_inputs() == 'drone pose missing or stale'
    finally:
        fixture.destroy()


def test_missing_drone_pose_reported():
    rclpy.init()
    fixture = _DetectionAdapterFixture()
    try:
        assert fixture.node._check_inputs() == 'drone pose missing or stale'
    finally:
        fixture.destroy()


def test_fresh_drone_pose_advances_past_pose_check():
    rclpy.init()
    fixture = _DetectionAdapterFixture()
    try:
        fixture.send_drone_pose()
        failure = fixture.node._check_inputs()
        assert failure is not None
        assert 'pose' not in failure
    finally:
        fixture.destroy()


def test_relay_skips_stale_drone_pose():
    rclpy.init()
    fixture = _DetectionAdapterFixture()
    try:
        sent = []
        fixture.node._tf_broadcaster.sendTransform = sent.append
        fixture.send_drone_pose()
        fixture.node._relay_drone_tf()
        assert len(sent) == 1
        fixture.now[0] += 5.0
        fixture.node._relay_drone_tf()
        assert len(sent) == 1
    finally:
        fixture.destroy()
