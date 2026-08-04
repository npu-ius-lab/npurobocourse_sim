import math
from dataclasses import dataclass
from typing import Tuple

MODES = ('fixed', 'line', 'loop')


@dataclass
class TrajectoryConfig:
    mode: str = 'fixed'
    start: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    yaw: float = 0.0
    speed: float = 0.3
    radius: float = 1.0
    seed: int = 0


def _phase(seed: int) -> float:
    return math.radians((seed % 360))


def sample(config: TrajectoryConfig, t: float) -> Tuple[float, float, float, float]:
    """Deterministic trajectory sample (x, y, z, yaw) at time t seconds."""
    if config.mode not in MODES:
        raise ValueError(f"unknown trajectory mode {config.mode!r}, expected one of {MODES}")
    sx, sy, sz = config.start

    if config.mode == 'fixed':
        return sx, sy, sz, config.yaw

    if config.mode == 'line':
        distance = config.speed * t
        return (
            sx + distance * math.cos(config.yaw),
            sy + distance * math.sin(config.yaw),
            sz,
            config.yaw,
        )

    if config.radius <= 0.0:
        raise ValueError(
            f'radius must be positive for loop mode, got {config.radius}')
    angle = _phase(config.seed) + (config.speed / config.radius) * t
    return (
        sx + config.radius * math.cos(angle),
        sy + config.radius * math.sin(angle),
        sz,
        angle + math.pi / 2.0,
    )
