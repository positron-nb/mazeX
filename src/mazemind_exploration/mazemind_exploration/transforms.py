"""Minimal homogeneous-transform helpers (avoids a tf2_geometry_msgs dependency)."""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def transform_to_matrix(tf) -> np.ndarray:
    """geometry_msgs/Transform (or TransformStamped) -> 4x4 matrix."""
    t = getattr(tf, 'transform', tf)
    q = t.rotation
    m = np.eye(4)
    m[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    m[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return m


def apply(m: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to (N, 3) points."""
    return points @ m[:3, :3].T + m[:3, 3]


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
