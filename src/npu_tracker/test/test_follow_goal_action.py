import math
import os
import threading
import time
import unittest

os.environ.setdefault('ROS_DOMAIN_ID', '35')

import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
import tf2_ros

from npu_tracker.publish_follow_goal import PublishFollowGoalNode


class MockNav2Server(Node):

    def __init__(self):
        super().__init__('mock_nav2_server')
        self.accept_goals = True
        self.reject_cancels = False
        self.cancel_response_delay = 0.0
        self.goal_requests = []
        self.cancel_requests = 0
        self._server = ActionServer(
            self, NavigateToPose, '/navigate_to_pose',
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            execute_callback=self._execute,
            callback_group=ReentrantCallbackGroup())

    def _on_goal(self, goal):
        self.goal_requests.append(
            (goal.pose.pose.position.x, goal.pose.pose.position.y))
        return GoalResponse.ACCEPT if self.accept_goals else GoalResponse.REJECT

    def _on_cancel(self, _goal_handle):
        self.cancel_requests += 1
        if self.cancel_response_delay > 0.0:
            time.sleep(self.cancel_response_delay)
        return CancelResponse.REJECT if self.reject_cancels else CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.05)


class FollowGoalProbe(Node):

    def __init__(self):
        super().__init__('follow_goal_probe')
        self.drone_xy = (5.0, 5.0)
        self.publish_enabled = True
        self._pose_pub = self.create_publisher(PoseStamped, '/npu/drone/pose', 10)
        self._tf_pub = tf2_ros.TransformBroadcaster(self)
        self.create_timer(0.05, self._publish)

    def _publish(self):
        if not self.publish_enabled:
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = self.drone_xy[0]
        pose.pose.position.y = self.drone_xy[1]
        pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)

        transform = TransformStamped()
        transform.header.stamp = pose.header.stamp
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'npu_robot/base_link'
        transform.transform.rotation.w = 1.0
        self._tf_pub.sendTransform(transform)


class TestPublishFollowGoal(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.server = MockNav2Server()
        cls.probe = FollowGoalProbe()
        cls.node = PublishFollowGoalNode(
            parameter_overrides=[
                Parameter('use_sim_time', value=False),
                Parameter('target_update_distance', value=0.3),
                Parameter('goal_update_interval_sec', value=1.0),
                Parameter('goal_retry_delay_sec', value=1.5),
                Parameter('pose_timeout_sec', value=2.0),
            ])
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.server)
        cls.executor.add_node(cls.probe)
        cls.executor.add_node(cls.node)
        cls._thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls._thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        cls.probe.destroy_node()
        cls.server.destroy_node()
        rclpy.shutdown()
        cls._thread.join(timeout=5.0)

    def _wait_for(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        return False

    def test_goal_projection_cancellation_rate_limit_and_rejection(self):
        # First goal is projected follow_distance short of the drone.
        self.assertTrue(self._wait_for(lambda: len(self.server.goal_requests) >= 1),
                        'no NavigateToPose goal submitted')
        x, y = self.server.goal_requests[0]
        expected = 5.0 - 5.0 * (1.5 / math.hypot(5.0, 5.0))
        self.assertAlmostEqual(x, expected, places=3)
        self.assertAlmostEqual(y, expected, places=3)

        # Rate limiting: an immediate target update must not replace the goal.
        self.probe.drone_xy = (8.0, 5.0)
        time.sleep(0.4)
        self.assertEqual(len(self.server.goal_requests), 1,
                         'goal replaced inside rate-limit interval')
        self.assertEqual(self.server.cancel_requests, 0)

        # After the interval the active goal is canceled before replacement.
        self.assertTrue(
            self._wait_for(lambda: len(self.server.goal_requests) >= 2, timeout=5.0),
            'replacement goal never submitted')
        self.assertGreaterEqual(self.server.cancel_requests, 1,
                                'active goal was not canceled before replacement')
        x, y = self.server.goal_requests[-1]
        expected_x = 8.0 - 8.0 * (1.5 / math.hypot(8.0, 5.0))
        expected_y = 5.0 - 5.0 * (1.5 / math.hypot(8.0, 5.0))
        self.assertAlmostEqual(x, expected_x, places=3)
        self.assertAlmostEqual(y, expected_y, places=3)

        # Rejection: backoff blocks immediate resend, then retries after delay.
        self.server.accept_goals = False
        self.probe.drone_xy = (12.0, 5.0)
        self.assertTrue(
            self._wait_for(lambda: len(self.server.goal_requests) >= 3, timeout=5.0),
            'goal request after target move never arrived')
        rejected_count = len(self.server.goal_requests)
        time.sleep(0.5)
        self.assertEqual(len(self.server.goal_requests), rejected_count,
                         'rejected goal was resent without retry delay')
        self.assertTrue(
            self._wait_for(lambda: len(self.server.goal_requests) > rejected_count,
                           timeout=5.0),
            'rejected goal was never retried after goal_retry_delay_sec')

        # Cancel rejection: a refused cancel drops the pending replacement
        # and backs off before resubmitting instead of thrashing goals.
        self.server.accept_goals = True
        goals_before = len(self.server.goal_requests)
        self.assertTrue(
            self._wait_for(lambda: len(self.server.goal_requests) > goals_before,
                           timeout=5.0),
            'no goal submitted after re-enabling acceptance')
        time.sleep(0.3)  # let the new goal be accepted and become active
        self.server.reject_cancels = True
        cancels_before = self.server.cancel_requests
        self.probe.drone_xy = (16.0, 5.0)
        self.assertTrue(
            self._wait_for(lambda: self.server.cancel_requests > cancels_before,
                           timeout=5.0),
            'replacement cancel was never requested')
        goals_after_cancel = len(self.server.goal_requests)
        time.sleep(0.8)
        self.assertEqual(len(self.server.goal_requests), goals_after_cancel,
                         'goal resubmitted during cancel-rejection backoff')
        self.assertTrue(
            self._wait_for(lambda: len(self.server.goal_requests) > goals_after_cancel,
                           timeout=5.0),
            'goal never resubmitted after cancel-rejection backoff')

        # Poses stale during cancel: the pending replacement is dropped and
        # a fresh goal is submitted once poses recover.
        self.server.reject_cancels = False
        self.server.cancel_response_delay = 2.5
        cancels_before = self.server.cancel_requests
        self.probe.drone_xy = (20.0, 5.0)
        self.assertTrue(
            self._wait_for(lambda: self.server.cancel_requests > cancels_before,
                           timeout=5.0),
            'stale-phase replacement cancel was never requested')
        self.probe.publish_enabled = False
        goals_before = len(self.server.goal_requests)
        time.sleep(3.0)
        self.assertEqual(len(self.server.goal_requests), goals_before,
                         'pending goal was submitted while poses were stale')
        self.probe.publish_enabled = True
        self.assertTrue(
            self._wait_for(lambda: len(self.server.goal_requests) > goals_before,
                           timeout=10.0),
            'node never recovered after poses came back')


if __name__ == '__main__':
    unittest.main()
