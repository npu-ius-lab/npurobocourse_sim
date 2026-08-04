import math
from typing import Optional, Sequence, Tuple

# Gazebo camera sensors look along +X of the sensor frame; ROS optical
# frames look along +Z with +X right and +Y down. This fixed rotation maps
# the Gazebo camera frame onto the ROS optical frame.
OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    qc = (-q[0], -q[1], -q[2], q[3])
    r = quat_multiply(quat_multiply(q, qv), qc)
    return (r[0], r[1], r[2])


def mount_quaternion(camera_rpy: Sequence[float]) -> Tuple[float, float, float, float]:
    """Orientation of the ROS optical frame in the drone base frame."""
    return quat_multiply(quat_from_rpy(*camera_rpy), quat_from_rpy(*OPTICAL_RPY))


def select_detection(detections, marker_id: int) -> Optional[object]:
    """Pick the configured marker from a detector-specific detection list."""
    for detection in detections:
        if detection.id == marker_id:
            return detection
    return None
