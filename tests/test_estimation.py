"""Estimation tests: pixel->bearing->world chain on synthetic data."""

import math

import pytest

import config
import estimation


class FakeDet:
    def __init__(self, u, bbox_w):
        self.centroid = (u, 120.0)
        self.bbox = (u - bbox_w / 2, 100, bbox_w, 40)


def test_focal_px():
    f = estimation.focal_px(config.CAMERA_WIDTH, config.CAMERA_FOV)
    assert f == pytest.approx((320 / 2) / math.tan(0.5), rel=1e-6)


def test_bearing_center_is_zero():
    phi = estimation.bearing_from_detection(160, 320, 1.0)
    assert phi == pytest.approx(0.0, abs=1e-9)


def test_bearing_left_positive():
    # object LEFT of image center -> positive bearing (left = +)
    phi = estimation.bearing_from_detection(80, 320, 1.0)
    assert phi > 0
    phi_r = estimation.bearing_from_detection(240, 320, 1.0)
    assert phi_r < 0
    assert abs(phi) == pytest.approx(abs(phi_r))


def test_estimate_position_with_lidar():
    pose = (0.0, 0.0, 0.0)
    n = 360
    angles = [-math.pi + i * 2 * math.pi / n for i in range(n)]
    scan = [4.0] * n
    # package at bearing 0, range 1.0 m
    idx = n // 2
    for i in range(idx - 4, idx + 5):
        scan[i] = 1.0
    x, y, phi, d = estimation.estimate_package_position(
        FakeDet(160, 40), pose, scan, angles)
    assert (x, y) == pytest.approx((1.0, 0.0), abs=0.02)
    assert d == pytest.approx(1.0)


def test_estimate_position_rotated_pose():
    pose = (1.0, 2.0, math.pi / 2)               # facing +y
    n = 360
    angles = [-math.pi + i * 2 * math.pi / n for i in range(n)]
    scan = [4.0] * n
    idx = n // 2
    for i in range(idx - 4, idx + 5):
        scan[i] = 2.0
    x, y, phi, d = estimation.estimate_package_position(
        FakeDet(160, 40), pose, scan, angles)
    assert (x, y) == pytest.approx((1.0, 4.0), abs=0.02)


def test_fallback_range_when_no_lidar():
    pose = (0.0, 0.0, 0.0)
    f = estimation.focal_px(320, 1.0)
    bbox_w = f * config.PKG_SIZE / 1.5           # would be 1.5 m away
    x, y, phi, d = estimation.estimate_package_position(
        FakeDet(160, bbox_w), pose, None, None)
    assert d == pytest.approx(1.5, rel=1e-6)
    assert x == pytest.approx(1.5, rel=1e-6)
