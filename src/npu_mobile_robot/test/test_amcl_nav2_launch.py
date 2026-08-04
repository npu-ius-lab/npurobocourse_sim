import math
import os
import time
import unittest

os.environ.setdefault('ROS_DOMAIN_ID', '33')
os.environ.setdefault('GZ_PARTITION', 'npu_test_amcl')

import launch
import launch_testing
import launch_testing.actions
import pytest
from launch.launch_description_sources import PythonLaunchDescriptionSource

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_ros import Buffer, TransformListener


WAIT_SEC = 180.0
GOAL_X = 3.5
GOAL_Y = -1.5


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
        _include(os.path.join(mobile, 'launch', 'amcl.launch.py')),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class NavProbe(Node):

    def __init__(self):
        super().__init__('nav_launch_probe')
        self.last_odom = None
        self.costmap_seen = False
        self.amcl_pose = None
        self.create_subscription(
            Odometry, '/npu/robot/odom', lambda msg: setattr(self, 'last_odom', msg), 10)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            lambda msg: setattr(self, 'costmap_seen', True), 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            lambda msg: setattr(self, 'amcl_pose', msg), 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def spin_until(self, predicate, timeout=WAIT_SEC):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if predicate():
                return True
        return False

    def lifecycle_state(self, node_name, timeout=30.0):
        client = self.create_client(GetState, f'/{node_name}/get_state')
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(GetState.Request())
        if not self.spin_until(lambda: future.done(), timeout):
            return None
        return future.result().current_state.label

    def wait_lifecycle_active(self, node_name, timeout=WAIT_SEC):
        deadline = time.monotonic() + timeout
        state = None
        while time.monotonic() < deadline:
            state = self.lifecycle_state(node_name, timeout=10.0)
            if state == 'active':
                return state
            time.sleep(1.0)
        return state


class TestAmclNav2Launch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = NavProbe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()

    def test_nav2_lifecycle_active(self):
        for node in ('map_server', 'amcl', 'controller_server',
                     'planner_server', 'behavior_server', 'bt_navigator'):
            state = self.probe.wait_lifecycle_active(node)
            self.assertEqual(state, 'active', f'{node} not active: {state}')

    def test_localization_tf(self):
        ok = self.probe.spin_until(lambda: self.probe.tf_buffer.can_transform(
            'world', 'npu_robot/base_link', rclpy.time.Time()))
        self.assertTrue(ok, 'world -> npu_robot/base_link TF unavailable')
        transform = self.probe.tf_buffer.lookup_transform(
            'world', 'npu_robot/odom', rclpy.time.Time())
        self.assertAlmostEqual(transform.transform.translation.x, 2.5, delta=0.6)
        self.assertAlmostEqual(transform.transform.translation.y, -1.5, delta=0.6)

    def test_navigate_to_pose_moves_robot(self):
        self.assertTrue(
            self.probe.nav_client.wait_for_server(timeout_sec=60.0),
            'NavigateToPose action server unavailable')
        self.assertTrue(
            self.probe.spin_until(lambda: self.probe.costmap_seen, 60.0),
            'global costmap never published')
        converged = self.probe.spin_until(
            lambda: self.probe.amcl_pose is not None
            and self.probe.amcl_pose.pose.covariance[0] < 0.25
            and self.probe.amcl_pose.pose.covariance[7] < 0.25,
            90.0)
        self.assertTrue(converged, 'AMCL did not converge to a confident pose')
        time.sleep(2.0)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'world'
        goal.pose.pose.position.x = GOAL_X
        goal.pose.pose.position.y = GOAL_Y
        goal.pose.pose.orientation.w = 1.0

        status = None
        for attempt in range(3):
            goal.pose.header.stamp = self.probe.get_clock().now().to_msg()
            send_future = self.probe.nav_client.send_goal_async(goal)
            self.assertTrue(self.probe.spin_until(lambda: send_future.done(), 30.0))
            goal_handle = send_future.result()
            self.assertTrue(goal_handle.accepted, 'NavigateToPose goal rejected')
            result_future = goal_handle.get_result_async()
            if self.probe.spin_until(lambda: result_future.done(), 150.0):
                status = result_future.result().status
                if status == 4:
                    break
            time.sleep(3.0)

        self.assertIsNotNone(self.probe.last_odom, 'no robot odometry received')
        moved_x = 2.5 + self.probe.last_odom.pose.pose.position.x
        moved_y = -1.5 + self.probe.last_odom.pose.pose.position.y
        displacement = math.hypot(moved_x - 2.5, moved_y + 1.5)
        self.assertGreater(displacement, 0.3, f'robot did not move (odom displacement {displacement:.3f} m)')

        self.assertEqual(status, 4, f'goal did not succeed after retries (status {status})')


def teardown_module():
    time.sleep(10)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -6, -15, 130, 137],
        )
