import math
import os
import threading
import time
import unittest
from unittest.mock import Mock

os.environ.setdefault('ROS_DOMAIN_ID', '35')

import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
import tf2_ros

from npu_tracker.publish_follow_goal import (
    ACTIVE,
    CANCELING,
    IDLE,
    SENDING,
    PublishFollowGoalNode,
)


class MockNav2Server(Node):

    def __init__(self):
        super().__init__('mock_nav2_server')
        self.accept_goals = True
        self.reject_cancels = False
        self.cancel_response_delay = 0.0
        self.goal_response_delay = 0.0
        self.goal_outcome = None
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
        if self.goal_response_delay > 0.0:
            time.sleep(self.goal_response_delay)
        return GoalResponse.ACCEPT if self.accept_goals else GoalResponse.REJECT

    def _on_cancel(self, _goal_handle):
        self.cancel_requests += 1
        if self.cancel_response_delay > 0.0:
            time.sleep(self.cancel_response_delay)
        return CancelResponse.REJECT if self.reject_cancels else CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        if self.goal_outcome == 'succeed':
            goal_handle.succeed()
            return NavigateToPose.Result()
        if self.goal_outcome == 'abort':
            goal_handle.abort()
            return NavigateToPose.Result()
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
        self.tf_publish_enabled = True
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
        if self.tf_publish_enabled:
            self._tf_pub.sendTransform(transform)


class TestPublishFollowGoal(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.server = MockNav2Server()
        self.probe = FollowGoalProbe()
        self.node = PublishFollowGoalNode(
            parameter_overrides=[
                Parameter('use_sim_time', value=False),
                Parameter('target_update_distance', value=0.3),
                Parameter('goal_update_interval_sec', value=1.0),
                Parameter('goal_retry_delay_sec', value=1.5),
                Parameter('pose_timeout_sec', value=2.0),
            ])
        self.executor = None
        self._thread = None

    def _start(self):
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.server)
        self.executor.add_node(self.probe)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def tearDown(self):
        if self.executor is not None:
            self.executor.shutdown()
        self.node.destroy_node()
        self.probe.destroy_node()
        self.server.destroy_node()
        rclpy.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    def _wait_for(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        return False

    def _wait_for_active(self):
        self.assertTrue(
            self._wait_for(lambda: self.node._phase == ACTIVE),
            'goal did not become active')

    def test_fixed_goal_succeeds_without_cancellation(self):
        self.server.goal_outcome = 'succeed'
        self._start()
        self.assertTrue(
            self._wait_for(lambda: self.node._last_result == 'succeeded'),
            'fixed goal did not succeed')
        self.assertEqual(self.node._phase, IDLE)
        self.assertIsNone(self.node._goal_handle)
        self.assertEqual(self.server.cancel_requests, 0)
        self.assertEqual(len(self.server.goal_requests), 1)
        time.sleep(0.3)
        self.assertEqual(self.node._last_result, 'succeeded')
        self.assertEqual(self.server.cancel_requests, 0)
        self.assertEqual(len(self.server.goal_requests), 1)

    def test_unexpected_abort_uses_failure_backoff(self):
        self.server.goal_outcome = 'abort'
        self._start()
        expected = f'failed (status {GoalStatus.STATUS_ABORTED})'
        self.assertTrue(
            self._wait_for(lambda: self.node._last_result == expected),
            'aborted goal was not classified as a failure')
        self.assertEqual(self.node._phase, IDLE)
        self.assertIsNone(self.node._goal_handle)
        self.assertEqual(self.server.cancel_requests, 0)
        self.assertTrue(self.node._replacer.in_backoff(self.node._wall_now()))
        goal_count = len(self.server.goal_requests)
        time.sleep(0.5)
        self.assertEqual(len(self.server.goal_requests), goal_count,
                         'aborted goal was resent during failure backoff')
        self.assertTrue(
            self._wait_for(lambda: len(self.server.goal_requests) > goal_count,
                           timeout=5.0),
            'aborted goal was not retried after failure backoff')

    def test_goal_projection_cancellation_rate_limit_and_rejection(self):
        self._start()
        self.assertTrue(self._wait_for(lambda: len(self.server.goal_requests) >= 1),
                        'no NavigateToPose goal submitted')
        self._wait_for_active()
        x, y = self.server.goal_requests[0]
        expected = 5.0 - 5.0 * (1.5 / math.hypot(5.0, 5.0))
        self.assertAlmostEqual(x, expected, places=3)
        self.assertAlmostEqual(y, expected, places=3)

        self.probe.drone_xy = (8.0, 5.0)
        time.sleep(0.4)
        self.assertEqual(len(self.server.goal_requests), 1,
                         'goal replaced inside rate-limit interval')
        self.assertEqual(self.server.cancel_requests, 0)

        self.assertTrue(
            self._wait_for(lambda: len(self.server.goal_requests) >= 2, timeout=5.0),
            'replacement goal never submitted')
        self._wait_for_active()
        self.assertGreaterEqual(self.server.cancel_requests, 1,
                                'active goal was not canceled before replacement')
        x, y = self.server.goal_requests[-1]
        expected_x = 8.0 - 8.0 * (1.5 / math.hypot(8.0, 5.0))
        expected_y = 5.0 - 5.0 * (1.5 / math.hypot(8.0, 5.0))
        self.assertAlmostEqual(x, expected_x, places=3)
        self.assertAlmostEqual(y, expected_y, places=3)

        self.server.accept_goals = False
        self.probe.drone_xy = (12.0, 5.0)
        self.assertTrue(
            self._wait_for(lambda: len(self.server.goal_requests) >= 3, timeout=5.0),
            'goal request after target move never arrived')
        rejected_count = len(self.server.goal_requests)
        time.sleep(0.5)
        self.assertEqual(len(self.server.goal_requests), rejected_count,
                         'rejected goal was resent without retry delay')
        self.server.accept_goals = True
        self.assertTrue(
            self._wait_for(lambda: len(self.server.goal_requests) > rejected_count,
                           timeout=5.0),
            'rejected goal was never retried after goal_retry_delay_sec')

    def test_cancel_rejection_retains_handle_and_stale_input_can_retry(self):
        self._start()
        self._wait_for_active()
        original_handle = self.node._goal_handle
        goal_count = len(self.server.goal_requests)
        self.server.reject_cancels = True
        cancels_before = self.server.cancel_requests
        self.probe.drone_xy = (8.0, 5.0)
        self.assertTrue(
            self._wait_for(lambda: self.server.cancel_requests > cancels_before,
                           timeout=5.0),
            'replacement cancel was never requested')
        self.assertTrue(
            self._wait_for(lambda: self.node._phase == ACTIVE
                           and not self.node._canceling_handles),
            'cancel rejection was not processed')
        self.assertIs(self.node._goal_handle, original_handle)
        self.assertEqual(len(self.server.goal_requests), goal_count)

        self.probe.publish_enabled = False
        self.server.reject_cancels = False
        cancels_before = self.server.cancel_requests
        self.assertTrue(
            self._wait_for(lambda: self.server.cancel_requests > cancels_before,
                           timeout=5.0),
            'stale input did not trigger another cancel')
        self.assertTrue(
            self._wait_for(lambda: self.node._phase == IDLE
                           and self.node._goal_handle is None),
            'accepted cancel did not release the original goal')
        self.assertEqual(len(self.server.goal_requests), goal_count)
        self.probe.publish_enabled = True
        self.assertTrue(
            self._wait_for(lambda: len(self.server.goal_requests) > goal_count,
                           timeout=10.0),
            'node did not recover after poses returned')

    def test_slow_goal_response_does_not_resend(self):
        self.server.goal_response_delay = 1.2
        self._start()
        self.assertTrue(
            self._wait_for(lambda: len(self.server.goal_requests) == 1),
            'slow goal request was not received')
        self.probe.drone_xy = (8.0, 5.0)
        time.sleep(0.4)
        self.assertEqual(self.node._phase, SENDING)
        self.assertEqual(len(self.server.goal_requests), 1,
                         'goal was resent while its response was pending')
        self._wait_for_active()
        self.assertEqual(len(self.server.goal_requests), 1)

    def test_robot_tf_stale_cancels_with_fresh_drone(self):
        self._start()
        self._wait_for_active()
        self.assertTrue(self._wait_for(lambda: self.node._robot_xy is not None))
        goal_count = len(self.server.goal_requests)
        cancels_before = self.server.cancel_requests
        self.probe.tf_publish_enabled = False
        self.assertTrue(
            self._wait_for(lambda: self.node._robot_xy is None
                           and 'stale' in self.node._tf_error, timeout=5.0),
            'robot TF was not marked stale')
        self.assertIsNotNone(self.node._fresh_drone_xy())
        self.assertTrue(
            self._wait_for(lambda: self.server.cancel_requests > cancels_before),
            'stale robot TF did not cancel the active goal')
        self.assertTrue(
            self._wait_for(lambda: self.node._phase == IDLE
                           and self.node._goal_handle is None),
            'robot TF cancellation did not finish')
        self.assertEqual(len(self.server.goal_requests), goal_count)
        self.assertIsNotNone(self.node._fresh_drone_xy())
        self.probe.tf_publish_enabled = True
        self.assertTrue(self._wait_for(lambda: self.node._robot_xy is not None))
        self.assertTrue(
            self._wait_for(lambda: len(self.server.goal_requests) > goal_count,
                           timeout=10.0),
            'same target did not resume after robot TF recovered')

    def test_result_request_failure_retries_during_cancel(self):
        now = [100.0]
        self.node._wall_now = lambda: now[0]
        result_future = Mock()
        handle = Mock()
        handle.accepted = True
        handle.goal_id.uuid = b'result-request'
        handle.get_result_async.side_effect = [RuntimeError('unavailable'), result_future]
        response_future = Mock()
        response_future.result.return_value = handle
        self.node._phase = SENDING
        self.node._on_goal_response(response_future)
        self.assertIs(self.node._goal_handle, handle)
        self.assertEqual(self.node._phase, ACTIVE)

        cancel_future = Mock()
        cancel_future.result.return_value.goals_canceling = [handle]
        self.node._phase = CANCELING
        self.node._on_cancel_done(cancel_future, handle)
        now[0] += 2.0
        self.node._tick()
        self.assertEqual(handle.get_result_async.call_count, 2)
        self.assertIs(self.node._result_pending, handle)
        self.assertIs(self.node._goal_handle, handle)

    def test_result_failure_retains_handle(self):
        handle = Mock()
        handle.goal_id.uuid = b'result-failure'
        future = Mock()
        future.result.side_effect = RuntimeError('unavailable')
        self.node._goal_handle = handle
        self.node._phase = ACTIVE
        self.node._on_result(future, handle)
        self.assertIs(self.node._goal_handle, handle)
        self.assertEqual(self.node._phase, ACTIVE)

    def test_stale_poses_during_cancel_drop_pending(self):
        self._start()
        self._wait_for_active()
        self.server.cancel_response_delay = 2.5
        cancels_before = self.server.cancel_requests
        self.probe.drone_xy = (8.0, 5.0)
        self.assertTrue(
            self._wait_for(lambda: self.server.cancel_requests > cancels_before,
                           timeout=5.0),
            'replacement cancel was never requested')
        self.probe.publish_enabled = False
        goals_before = len(self.server.goal_requests)
        time.sleep(3.0)
        self.assertEqual(len(self.server.goal_requests), goals_before,
                         'pending goal was submitted while poses were stale')
        self.server.cancel_response_delay = 0.0
        self.probe.publish_enabled = True
        self.assertTrue(
            self._wait_for(lambda: len(self.server.goal_requests) > goals_before,
                           timeout=10.0),
            'node never recovered after poses came back')


if __name__ == '__main__':
    unittest.main()
