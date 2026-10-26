import os
import signal
import subprocess
import sys
import time
import unittest

os.environ.setdefault('ROS_DOMAIN_ID', '37')

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
import tf2_ros


def _is_nonzero(msg):
    return any(abs(value) > 1e-9 for value in (
        msg.linear.x,
        msg.linear.y,
        msg.linear.z,
        msg.angular.x,
        msg.angular.y,
        msg.angular.z,
    ))


class FollowExitProbe(Node):

    def __init__(self, round_index):
        super().__init__(f'follow_exit_probe_{round_index}')
        self.commands = []
        self._pose_pub = self.create_publisher(PoseStamped, '/npu/drone/pose', 10)
        self._tf_pub = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(
            Twist, '/npu/robot/cmd_vel', self._on_command, 10)
        self.create_timer(0.05, self._publish_inputs)

    def _publish_inputs(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = 5.0
        pose.pose.position.y = 5.0
        pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)

        transform = TransformStamped()
        transform.header.stamp = pose.header.stamp
        transform.header.frame_id = 'npu_robot/odom'
        transform.child_frame_id = 'npu_robot/base_link'
        transform.transform.rotation.w = 1.0
        self._tf_pub.sendTransform(transform)

    def _on_command(self, msg):
        self.commands.append((time.monotonic(), msg))

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.005)
            if predicate():
                return True
        return predicate()


class TestFollowExit(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def _run_round(self, round_index):
        probe = FollowExitProbe(round_index)
        process = subprocess.Popen(
            [
                sys.executable,
                '-m',
                'npu_tracker.follow_drone_tf',
                '--ros-args',
                '-p',
                'use_sim_time:=false',
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            got_nonzero = probe.spin_until(
                lambda: process.poll() is not None or any(
                    _is_nonzero(msg) for _, msg in probe.commands),
                timeout=10.0,
            )
            if not got_nonzero or process.poll() is not None:
                if process.poll() is None:
                    process.kill()
                output = process.communicate(timeout=2.0)[0]
                self.fail(
                    f'round {round_index}: no non-zero command before SIGINT\n'
                    f'{output}')

            probe.commands.clear()
            signal_at = time.monotonic()
            process.send_signal(signal.SIGINT)
            exit_deadline = signal_at + 5.0
            while process.poll() is None and time.monotonic() < exit_deadline:
                rclpy.spin_once(probe, timeout_sec=0.005)
            if process.poll() is None:
                process.kill()
                output = process.communicate(timeout=2.0)[0]
                self.fail(f'round {round_index}: node did not exit after SIGINT\n'
                          f'{output}')
            output = process.communicate(timeout=2.0)[0]

            settle_deadline = time.monotonic() + 0.1
            while time.monotonic() < settle_deadline:
                rclpy.spin_once(probe, timeout_sec=0.005)

            zero_indices = [
                index for index, (stamp, msg) in enumerate(probe.commands)
                if stamp >= signal_at and not _is_nonzero(msg)
            ]
            self.assertTrue(
                zero_indices,
                f'round {round_index}: no final zero Twist after SIGINT\n{output}')
            final_zero_index = zero_indices[0]
            final_zero_at = probe.commands[final_zero_index][0]
            self.assertLess(
                final_zero_at - signal_at,
                0.5,
                f'round {round_index}: final zero Twist arrived too late\n{output}',
            )
            later_nonzero = [
                msg for _, msg in probe.commands[final_zero_index + 1:]
                if _is_nonzero(msg)
            ]
            self.assertFalse(
                later_nonzero,
                f'round {round_index}: non-zero Twist followed final zero\n{output}',
            )
            self.assertEqual(
                process.returncode, 0,
                f'round {round_index}: abnormal node exit\n{output}')
            self.assertNotIn(
                'final zero command',
                output.lower(),
                f'round {round_index}: final zero publish/ACK failed\n{output}',
            )
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate(timeout=2.0)
            probe.destroy_node()

    def test_five_sigint_exits_deliver_final_zero(self):
        for round_index in range(1, 6):
            with self.subTest(round=round_index):
                self._run_round(round_index)


if __name__ == '__main__':
    unittest.main()
