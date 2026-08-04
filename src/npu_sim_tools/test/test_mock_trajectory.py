import math

import pytest

from npu_sim_tools.mock_trajectory import TrajectoryConfig, sample


def test_fixed_mode_stays_at_start():
    config = TrajectoryConfig(mode='fixed', start=(1.0, 2.0, 0.5), yaw=0.3)
    for t in (0.0, 5.0, 100.0):
        assert sample(config, t) == (1.0, 2.0, 0.5, 0.3)


def test_line_mode_advances_along_start_yaw():
    config = TrajectoryConfig(mode='line', start=(0.0, 0.0, 1.0),
                              yaw=math.pi / 2.0, speed=0.5)
    x, y, z, yaw = sample(config, 4.0)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(2.0)
    assert z == pytest.approx(1.0)
    assert yaw == pytest.approx(math.pi / 2.0)


def test_loop_mode_stays_on_circle():
    config = TrajectoryConfig(mode='loop', start=(1.0, 1.0, 0.8),
                              speed=0.3, radius=1.5, seed=0)
    for t in (0.0, 1.7, 12.3):
        x, y, z, _ = sample(config, t)
        assert math.hypot(x - 1.0, y - 1.0) == pytest.approx(1.5)
        assert z == pytest.approx(0.8)


def test_loop_mode_yaw_follows_tangent_direction():
    config = TrajectoryConfig(mode='loop', start=(0.0, 0.0, 1.0),
                              speed=0.3, radius=1.5, seed=0)
    _, _, _, yaw0 = sample(config, 0.0)
    assert yaw0 == pytest.approx(math.pi / 2.0)
    _, _, _, yaw1 = sample(config, 1.25 * math.pi)
    assert yaw1 == pytest.approx(3.0 * math.pi / 4.0)


def test_loop_mode_zero_radius_rejected():
    config = TrajectoryConfig(mode='loop', radius=0.0)
    with pytest.raises(ValueError):
        sample(config, 1.0)


def test_loop_mode_is_deterministic_for_same_seed():
    config_a = TrajectoryConfig(mode='loop', seed=7)
    config_b = TrajectoryConfig(mode='loop', seed=7)
    assert sample(config_a, 3.3) == sample(config_b, 3.3)


def test_unknown_mode_rejected():
    config = TrajectoryConfig(mode='spiral')
    with pytest.raises(ValueError):
        sample(config, 0.0)
