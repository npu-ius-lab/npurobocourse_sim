import os

os.environ.setdefault('ROS_DOMAIN_ID', '39')

import rclpy

from geometry_msgs.msg import PoseStamped

from npu_sim_bringup.display_logic import DisplayConfig, validate_external_pose
from npu_sim_bringup.external_pose_display import ExternalPoseDisplayAdapter


CONFIG = DisplayConfig()
QUAT = (0.0, 0.0, 0.0, 1.0)


def test_valid_pose_accepted():
    assert validate_external_pose((1.0, -2.0, 1.5), QUAT, 'world', CONFIG) is None


def test_wrong_frame_rejected():
    reason = validate_external_pose((0.0, 0.0, 1.0), QUAT, 'map', CONFIG)
    assert reason is not None
    assert 'map' in reason


def test_non_finite_pose_rejected():
    assert validate_external_pose((float('nan'), 0.0, 1.0), QUAT, 'world', CONFIG)
    assert validate_external_pose((0.0, 0.0, float('inf')), QUAT, 'world', CONFIG)


def test_degenerate_quaternion_rejected():
    assert validate_external_pose((0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 0.0),
                                  'world', CONFIG)


def test_out_of_bounds_rejected():
    assert validate_external_pose((0.0, 0.0, -0.5), QUAT, 'world', CONFIG)
    assert validate_external_pose((0.0, 0.0, 99.0), QUAT, 'world', CONFIG)
    assert validate_external_pose((99.0, 0.0, 1.0), QUAT, 'world', CONFIG)


def test_boundary_pose_accepted():
    assert validate_external_pose(
        (CONFIG.bounds_x[0], CONFIG.bounds_y[1], CONFIG.bounds_z[0]),
        QUAT, 'world', CONFIG) is None


def test_set_pose_failure_is_retried_without_recording_sent():
    class RejectedGz:
        calls = 0

        def request(self, *args):
            self.calls += 1
            return True, type('Response', (), {'data': False})()

    rclpy.init()
    node = ExternalPoseDisplayAdapter()
    fake = RejectedGz()
    node._gz_node = fake
    try:
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.pose.position.z = 1.0
        pose.pose.orientation.w = 1.0
        node._on_pose(pose)
        node._tick()
        node._tick()
        assert node._set_pose_service.endswith('/set_pose/blocking')
        assert fake.calls == 2
        assert node._last_sent is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
