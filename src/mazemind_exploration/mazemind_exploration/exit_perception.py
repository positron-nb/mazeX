"""Camera + LiDAR exit-marker perception (no ground truth).

Mechanism
---------
1. The camera image is thresholded in HSV for the exit marker's saturated
   green; the largest blob above ``min_area_frac`` of the image is kept.
2. LiDAR points are transformed into the camera optical frame and projected
   onto image columns with the pinhole model. Points whose column falls
   inside the blob's horizontal extent are the marker surface.
3. Their median gives the marker position; a line fit gives the wall
   direction, whose normal (pointing toward the robot) is the approach axis.
4. The apparent blob width must agree with the width expected at the LiDAR
   range (rejects tiny distant specks and implausibly large blobs).
5. :class:`DetectionValidator` only accepts a marker after several
   consistent observations within a short time window.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class ColorThreshold:
    h_min: int = 45      # OpenCV hue range is 0..179; pure green is 60
    h_max: int = 85
    s_min: int = 120
    v_min: int = 60


@dataclass
class Blob:
    u_min: int
    u_max: int
    v_min: int
    v_max: int
    area: int
    centroid: Tuple[float, float]

    @property
    def width(self) -> int:
        return self.u_max - self.u_min + 1


def find_marker_blob(bgr: np.ndarray, thresh: ColorThreshold, min_area_frac: float = 0.002
                     ) -> Tuple[Optional[Blob], np.ndarray]:
    """Return the largest marker-coloured blob (or None) and the binary mask."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lower = np.array([thresh.h_min, thresh.s_min, thresh.v_min], dtype=np.uint8)
    upper = np.array([thresh.h_max, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return None, mask
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[k, cv2.CC_STAT_AREA])
    if area < min_area_frac * mask.shape[0] * mask.shape[1]:
        return None, mask
    x, y, w, h = (int(stats[k, i]) for i in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                                             cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
    return Blob(x, x + w - 1, y, y + h - 1, area,
                (float(centroids[k][0]), float(centroids[k][1]))), mask


def points_in_blob_columns(points_cam: np.ndarray, fx: float, cx: float, blob: Blob,
                           margin_px: int = 2, max_range: float = 4.0) -> np.ndarray:
    """Boolean mask of camera-optical-frame points that project into the blob's columns."""
    z = points_cam[:, 2]
    in_front = z > 0.05
    u = np.full(z.shape, -1e9)
    u[in_front] = fx * points_cam[in_front, 0] / z[in_front] + cx
    rng = np.linalg.norm(points_cam, axis=1)
    # Trim edge columns (mixed pixels) only for wide blobs: a narrow blob from an oblique
    # or distant view spans just a few 1-degree LiDAR bearings, and trimming left too few.
    margin = margin_px if blob.width >= 10 * max(margin_px, 1) else 0
    lo, hi = blob.u_min + margin, blob.u_max - margin
    return in_front & (u >= lo) & (u <= hi) & (rng <= max_range)


def fit_marker_pose(points_xy: np.ndarray, viewer_xy: Sequence[float]
                    ) -> Tuple[Tuple[float, float], float, float]:
    """Marker centre, outward normal yaw (toward the viewer) and point spread along the wall."""
    center = np.median(points_xy, axis=0)
    centered = points_xy - points_xy.mean(axis=0)
    if points_xy.shape[0] >= 2 and np.ptp(centered, axis=0).max() > 1e-6:
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
        direction = vt[0]
    else:
        direction = np.array([1.0, 0.0])
    normal = np.array([-direction[1], direction[0]])
    to_viewer = np.asarray(viewer_xy, dtype=float) - center
    if float(np.dot(normal, to_viewer)) < 0:
        normal = -normal
    spread = float(np.ptp(centered @ direction)) if points_xy.shape[0] >= 2 else 0.0
    return (float(center[0]), float(center[1])), math.atan2(normal[1], normal[0]), spread


def expected_width_px(fx: float, marker_width: float, rng: float) -> float:
    return fx * marker_width / max(rng, 1e-3)


def width_consistent(blob_width_px: float, fx: float, marker_width: float, rng: float,
                     min_ratio: float = 0.15, max_ratio: float = 1.8) -> bool:
    """Partial occlusion/oblique views shrink the blob; nothing should make it much larger."""
    ratio = blob_width_px / expected_width_px(fx, marker_width, rng)
    return min_ratio <= ratio <= max_ratio


@dataclass
class Observation:
    stamp: float
    x: float
    y: float
    normal_yaw: float


class DetectionValidator:
    """Confirms a marker after ``min_count`` observations within ``window`` seconds
    that all lie within ``cluster_radius`` metres of their mean."""

    def __init__(self, min_count: int = 3, window: float = 4.0, cluster_radius: float = 0.3):
        self.min_count = min_count
        self.window = window
        self.cluster_radius = cluster_radius
        self._obs: Deque[Observation] = deque()
        self.confirmed: Optional[Observation] = None
        self._confirmed_n = 0

    def add(self, obs: Observation) -> Optional[Observation]:
        """Add an observation; return the (updated) confirmed estimate, if any."""
        self._obs.append(obs)
        while self._obs and obs.stamp - self._obs[0].stamp > self.window:
            self._obs.popleft()
        if self.confirmed is not None:
            # refine with consistent observations only (running mean)
            if math.hypot(obs.x - self.confirmed.x, obs.y - self.confirmed.y) <= self.cluster_radius:
                self._confirmed_n += 1
                n = self._confirmed_n
                c = self.confirmed
                yaw = math.atan2((n - 1) * math.sin(c.normal_yaw) + math.sin(obs.normal_yaw),
                                 (n - 1) * math.cos(c.normal_yaw) + math.cos(obs.normal_yaw))
                self.confirmed = Observation(obs.stamp, c.x + (obs.x - c.x) / n,
                                             c.y + (obs.y - c.y) / n, yaw)
            return self.confirmed
        if len(self._obs) < self.min_count:
            return None
        recent = list(self._obs)[-self.min_count:]
        mx = sum(o.x for o in recent) / len(recent)
        my = sum(o.y for o in recent) / len(recent)
        if all(math.hypot(o.x - mx, o.y - my) <= self.cluster_radius for o in recent):
            yaw = math.atan2(sum(math.sin(o.normal_yaw) for o in recent),
                             sum(math.cos(o.normal_yaw) for o in recent))
            self.confirmed = Observation(obs.stamp, mx, my, yaw)
            self._confirmed_n = len(recent)
        return self.confirmed
