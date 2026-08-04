import math

from geometry_msgs.msg import Twist

from npu_sim_bringup.command_gate import CommandGate, CommandLimits


def _twist(x=0.0, y=0.0, z=0.0, yaw=0.0):
    msg = Twist()
    msg.linear.x = x
    msg.linear.y = y
    msg.linear.z = z
    msg.angular.z = yaw
    return msg


def _gate():
    return CommandGate(
        limits=CommandLimits(max_linear_xy=2.0, max_vertical=1.0, max_yaw_rate=1.0),
        command_timeout_sec=0.5,
    )


def test_valid_command_passes_unchanged():
    gate = _gate()
    out, err = gate.validate(_twist(0.5, -0.5, 0.3, 0.4), now=1.0)
    assert err is None
    assert out.linear.x == 0.5
    assert out.linear.y == -0.5
    assert out.linear.z == 0.3
    assert out.angular.z == 0.4


def test_command_is_clamped_to_limits():
    gate = _gate()
    out, err = gate.validate(_twist(10.0, -10.0, 5.0, -7.0), now=1.0)
    assert err is None
    assert out.linear.x == 2.0
    assert out.linear.y == -2.0
    assert out.linear.z == 1.0
    assert out.angular.z == -1.0


def test_non_finite_command_rejected():
    gate = _gate()
    for value in (math.nan, math.inf, -math.inf):
        out, err = gate.validate(_twist(x=value), now=1.0)
        assert out is None
        assert err is not None


def test_rejected_command_does_not_refresh_timeout():
    gate = _gate()
    out, err = gate.validate(_twist(0.1), now=1.0)
    assert err is None
    out, err = gate.validate(_twist(x=math.nan), now=1.2)
    assert out is None
    zero, timed_out = gate.watchdog(now=1.6)
    assert zero is not None
    assert timed_out
    assert zero.linear.x == 0.0


def test_watchdog_zeroes_stale_command_once():
    gate = _gate()
    gate.validate(_twist(1.0), now=1.0)
    zero, timed_out = gate.watchdog(now=1.4)
    assert zero is None
    assert not timed_out
    zero, timed_out = gate.watchdog(now=1.7)
    assert zero is not None
    assert timed_out
    zero, timed_out = gate.watchdog(now=1.8)
    assert zero is None
    assert timed_out


def test_command_after_timeout_is_accepted():
    gate = _gate()
    gate.validate(_twist(1.0), now=1.0)
    gate.watchdog(now=2.0)
    out, err = gate.validate(_twist(0.2), now=2.1)
    assert err is None
    assert out.linear.x == 0.2
    assert not gate.is_timed_out(now=2.1)


def test_no_command_means_timed_out():
    gate = _gate()
    assert gate.is_timed_out(now=0.0)
    zero, timed_out = gate.watchdog(now=0.0)
    assert zero is None
    assert timed_out
