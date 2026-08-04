import os

os.environ.setdefault('ROS_DOMAIN_ID', '38')

import pytest
import rclpy
from rclpy.parameter import Parameter

from npu_tracker.mission import MissionNode


def test_non_triplet_waypoints_rejected_at_construction():
    rclpy.init()
    try:
        with pytest.raises(ValueError, match='xyz triplets'):
            MissionNode(parameter_overrides=[
                Parameter('waypoints', value=[1.0, 2.0])])
    finally:
        rclpy.shutdown()


def test_valid_waypoints_construct():
    rclpy.init()
    node = None
    try:
        node = MissionNode(parameter_overrides=[
            Parameter('waypoints', value=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0])])
        assert node._mission.config.waypoints == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
