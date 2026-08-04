import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class DisplayConfig:
    world_frame: str = 'world'
    entity_name: str = 'npu_drone_display'
    bounds_x: Tuple[float, float] = (-12.0, 12.0)
    bounds_y: Tuple[float, float] = (-12.0, 12.0)
    bounds_z: Tuple[float, float] = (0.0, 6.0)


def validate_external_pose(
    position: Tuple[float, float, float],
    quaternion: Tuple[float, float, float, float],
    frame_id: str,
    config: DisplayConfig,
) -> Optional[str]:
    """Return None when the external pose may drive the display entity,
    otherwise a human-readable rejection reason."""
    if frame_id != config.world_frame:
        return (f'pose frame {frame_id!r} != {config.world_frame!r}')

    values = list(position) + list(quaternion)
    if not all(math.isfinite(v) for v in values):
        return 'pose contains non-finite values'

    norm = math.sqrt(sum(v * v for v in quaternion))
    if norm < 1e-3:
        return 'pose quaternion is degenerate'

    axes = (config.bounds_x, config.bounds_y, config.bounds_z)
    for axis, (low, high) in zip(('x', 'y', 'z'), axes):
        value = position['xyz'.index(axis)]
        if not (low <= value <= high):
            return f'pose {axis}={value:.3f} outside bounds [{low}, {high}]'

    return None
