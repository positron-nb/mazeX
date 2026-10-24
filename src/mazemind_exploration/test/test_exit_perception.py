import math

import numpy as np
import pytest

from mazemind_exploration.exit_perception import (ColorThreshold, DetectionValidator, Observation,
                                                  find_marker_blob, fit_marker_pose,
                                                  points_in_blob_columns, width_consistent)


def scene(with_marker=True):
    img = np.full((240, 320, 3), (185, 184, 180), dtype=np.uint8)   # grey walls (BGR)
    img[150:, :] = (95, 90, 90)                                       # floor
    if with_marker:
        img[100:160, 120:200] = (25, 217, 13)                         # green marker (BGR)
    return img


def test_blob_found_on_green_marker():
    blob, mask = find_marker_blob(scene(), ColorThreshold())
    assert blob is not None
    assert (blob.u_min, blob.u_max) == (120, 199)
    assert blob.area == pytest.approx(60 * 80, rel=0.05)


def test_no_blob_without_marker_and_small_specks_ignored():
    assert find_marker_blob(scene(False), ColorThreshold())[0] is None
    img = scene(False)
    img[10:13, 10:13] = (25, 217, 13)
    assert find_marker_blob(img, ColorThreshold(), min_area_frac=0.002)[0] is None


def test_lidar_points_projected_into_blob_columns():
    blob, _ = find_marker_blob(scene(), ColorThreshold())
    fx, cx = 200.0, 160.0
    # points in the camera optical frame (x right, z forward), 2 m ahead
    xs = np.linspace(-1.0, 1.0, 41)
    pts = np.stack([xs, np.zeros_like(xs), np.full_like(xs, 2.0)], axis=1)
    sel = points_in_blob_columns(pts, fx, cx, blob)
    u = fx * xs / 2.0 + cx
    assert sel.any()
    assert np.all((u[sel] >= blob.u_min) & (u[sel] <= blob.u_max))
    behind = pts.copy()
    behind[:, 2] = -2.0
    assert not points_in_blob_columns(behind, fx, cx, blob).any()


def test_fit_marker_pose_normal_points_to_viewer():
    ys = np.linspace(-0.3, 0.3, 10)
    pts = np.stack([np.full_like(ys, 3.0), ys], axis=1)     # wall at x = 3
    (cx, cy), yaw, spread = fit_marker_pose(pts, viewer_xy=(0.0, 0.0))
    assert cx == pytest.approx(3.0) and cy == pytest.approx(0.0, abs=1e-9)
    assert abs(math.remainder(yaw - math.pi, 2 * math.pi)) < 1e-6
    assert spread == pytest.approx(0.6)


def test_width_consistency():
    fx = 200.0
    assert width_consistent(60, fx, 0.6, 2.0)       # expected 60 px
    assert width_consistent(20, fx, 0.6, 2.0)       # partially occluded
    assert not width_consistent(200, fx, 0.6, 2.0)  # far too wide for that range
    assert not width_consistent(3, fx, 0.6, 2.0)


def test_validator_requires_consistent_observations():
    v = DetectionValidator(min_count=3, window=4.0, cluster_radius=0.3)
    assert v.add(Observation(0.0, 1.0, 1.0, 0.0)) is None
    assert v.add(Observation(0.5, 3.0, 3.0, 0.0)) is None      # inconsistent
    assert v.add(Observation(1.0, 1.05, 1.0, 0.0)) is None
    assert v.add(Observation(1.5, 1.0, 1.05, 0.0)) is None     # last 3 include the outlier
    c = v.add(Observation(2.0, 1.02, 1.0, 0.0))
    assert c is not None and c.x == pytest.approx(1.023, abs=0.01)
    # outliers after confirmation do not move the estimate
    c2 = v.add(Observation(2.5, 5.0, 5.0, 0.0))
    assert c2.x == pytest.approx(c.x)


def test_validator_window_expires_old_observations():
    v = DetectionValidator(min_count=3, window=2.0, cluster_radius=0.3)
    v.add(Observation(0.0, 1.0, 1.0, 0.0))
    v.add(Observation(1.0, 1.0, 1.0, 0.0))
    assert v.add(Observation(10.0, 1.0, 1.0, 0.0)) is None


def test_narrow_blob_keeps_edge_columns():
    """A 13 px wide blob (oblique view) must not lose its edge columns to trimming."""
    from mazemind_exploration.exit_perception import Blob
    blob = Blob(u_min=150, u_max=162, v_min=100, v_max=140, area=500, centroid=(156.0, 120.0))
    fx, cx = 210.0, 160.0
    # three LiDAR returns at 2 m whose columns fall at u = 150, 156, 162
    us = np.array([150.0, 156.0, 162.0])
    pts = np.stack([(us - cx) * 2.0 / fx, np.zeros(3), np.full(3, 2.0)], axis=1)
    assert points_in_blob_columns(pts, fx, cx, blob).sum() == 3
