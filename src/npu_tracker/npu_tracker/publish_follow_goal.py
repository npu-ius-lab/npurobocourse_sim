import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import tf2_ros

from npu_tracker.follow_goal_logic import (
    FollowGoalConfig,
    GoalReplacer,
    project_target,
)


class PublishFollowGoalNode(Node):

    def __init__(self, **kwargs):
        super().__init__('publish_follow_goal', **kwargs)

        # Simulation-only workspace: default to sim time so bare `ros2 run`
        # matches the launch-file behavior (README interface convention).
        # rclpy's TimeSource already declared use_sim_time (default False);
        # an explicit override (launch file, tests) always wins.
        if 'use_sim_time' not in getattr(self, '_parameter_overrides', {}):
            self.set_parameters(
                [Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.declare_parameter('follow_distance', 1.5)
        self.declare_parameter('target_update_distance', 0.3)
        self.declare_parameter('goal_update_interval_sec', 2.0)
        self.declare_parameter('goal_retry_delay_sec', 2.0)
        self.declare_parameter('pose_timeout_sec', 1.0)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('base_frame', 'npu_robot/base_link')

        config = FollowGoalConfig(
            follow_distance=self.get_parameter('follow_distance').value,
            target_update_distance=self.get_parameter('target_update_distance').value,
            goal_update_interval_sec=self.get_parameter('goal_update_interval_sec').value,
            goal_retry_delay_sec=self.get_parameter('goal_retry_delay_sec').value,
        )
        self._replacer = GoalReplacer(config)
        self._follow_distance = config.follow_distance
        self._pose_timeout = self.get_parameter('pose_timeout_sec').value
        self._world_frame = self.get_parameter('world_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        self._last_pose = None
        self._last_pose_wall = None
        self._robot_xy = None
        self._tf_error = ''
        self._goal_handle = None
        self._pending_target = None
        self._canceling_handles = set()
        self._last_result = 'none'

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self._goal_pose_pub = self.create_publisher(
            PoseStamped, '/npu/robot/goal_pose', 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.create_subscription(PoseStamped, '/npu/drone/pose', self._on_pose, 10)

        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info('publish_follow_goal started')

    def _wall_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != self._world_frame:
            self.get_logger().warn(
                f'ignoring pose in frame {msg.header.frame_id!r}, '
                f'expected {self._world_frame!r}')
            return
        self._last_pose = msg
        self._last_pose_wall = self._wall_now()

    def _fresh_drone_xy(self):
        if self._last_pose is None or self._last_pose_wall is None:
            return None
        if self._wall_now() - self._last_pose_wall > self._pose_timeout:
            return None
        return (self._last_pose.pose.position.x, self._last_pose.pose.position.y)

    def _update_robot_xy(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._world_frame, self._base_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self._robot_xy = None
            self._tf_error = f'robot TF unavailable: {exc}'
            return
        self._tf_error = ''
        self._robot_xy = (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _tick(self) -> None:
        drone_xy = self._fresh_drone_xy()
        self._update_robot_xy()
        if drone_xy is None or self._robot_xy is None:
            self._cancel_active_goal()
            return
        if not self._action_client.server_is_ready():
            return

        target = project_target(drone_xy, self._robot_xy, self._follow_distance)
        approved = self._replacer.update(target, self._wall_now())
        if approved is None:
            return

        if self._pending_target is not None:
            return
        if self._goal_handle is not None:
            self._pending_target = approved
            goal_handle = self._goal_handle
            self._canceling_handles.add(bytes(goal_handle.goal_id.uuid))
            goal_handle.cancel_goal_async().add_done_callback(
                lambda future, gh=goal_handle: self._on_cancel_done(future, gh))
        else:
            self._send_goal(approved)

    def _cancel_active_goal(self) -> None:
        goal_handle = self._goal_handle
        if goal_handle is None or self._pending_target is not None:
            return
        self._goal_handle = None
        self._canceling_handles.add(bytes(goal_handle.goal_id.uuid))
        self.get_logger().warn(
            'cancelling active goal: drone pose or robot pose unavailable')
        goal_handle.cancel_goal_async().add_done_callback(
            lambda future, gh=goal_handle: self._on_cancel_done(future, gh))

    def _send_goal(self, target) -> None:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._world_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = target[0]
        goal.pose.pose.position.y = target[1]
        goal.pose.pose.orientation.z = math.sin(target[2] / 2.0)
        goal.pose.pose.orientation.w = math.cos(target[2] / 2.0)

        self._goal_pose_pub.publish(goal.pose)
        self._replacer.on_sent(target, self._wall_now())
        self.get_logger().info(
            f'submitting NavigateToPose goal ({target[0]:.2f}, {target[1]:.2f})')
        self._action_client.send_goal_async(goal).add_done_callback(
            self._on_goal_response)

    def _on_cancel_done(self, future, goal_handle) -> None:
        if self._goal_handle is goal_handle:
            self._goal_handle = None
        response = future.result()
        if response is None:
            self.get_logger().warn('cancel request failed: no response')
            self._pending_target = None
            self._replacer.on_rejected_or_failed(self._wall_now())
            return
        accepted = any(bytes(info.goal_id.uuid) == bytes(goal_handle.goal_id.uuid)
                       for info in response.goals_canceling)
        pending, self._pending_target = self._pending_target, None
        if pending is None:
            return
        if not accepted:
            self.get_logger().warn(
                f'cancel rejected (return_code {response.return_code}), '
                'backing off before retrying')
            self._replacer.on_rejected_or_failed(self._wall_now())
            return
        if self._fresh_drone_xy() is None or self._robot_xy is None:
            self.get_logger().warn('poses went stale during cancel, dropping pending goal')
            self._replacer.on_rejected_or_failed(self._wall_now())
            return
        self._send_goal(pending)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._last_result = 'rejected'
            self.get_logger().warn('NavigateToPose goal rejected')
            self._replacer.on_rejected_or_failed(self._wall_now())
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda future, gh=goal_handle: self._on_result(future, gh))

    def _on_result(self, future, goal_handle) -> None:
        status = future.result().status
        if self._goal_handle is goal_handle:
            self._goal_handle = None
        replaced = bytes(goal_handle.goal_id.uuid) in self._canceling_handles
        self._canceling_handles.discard(bytes(goal_handle.goal_id.uuid))
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._last_result = 'succeeded'
        elif replaced:
            # Nav2 terminates the preempted goal as ABORTED or CANCELED when
            # we replace it; that is expected, not a failure.
            self._last_result = 'replaced'
        else:
            self._last_result = f'failed (status {status})'
            self.get_logger().warn(
                f'NavigateToPose goal ended with status {status}')
            self._replacer.on_rejected_or_failed(self._wall_now())

    def _publish_diagnostics(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'publish_follow_goal/goal'
        status.hardware_id = 'npu_robot'
        now = self._wall_now()
        if self._fresh_drone_xy() is None:
            status.level = DiagnosticStatus.WARN
            status.message = 'drone pose missing or stale, active goal cancelled'
        elif self._tf_error:
            status.level = DiagnosticStatus.WARN
            status.message = self._tf_error
        elif not self._action_client.server_is_ready():
            status.level = DiagnosticStatus.WARN
            status.message = 'waiting for NavigateToPose action server'
        elif self._replacer.in_backoff(now):
            status.level = DiagnosticStatus.WARN
            status.message = f'retry backoff after {self._last_result}'
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'tracking (last result: {self._last_result})'
        array.status.append(status)
        self._diag_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = PublishFollowGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
