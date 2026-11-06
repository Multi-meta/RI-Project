from detector import Detection
import estimation
import pytest
import math

def test_rear_camera_bearing():
    # Center of rear camera should point directly behind (pi rad = 180 deg)
    phi_center = estimation.bearing_from_detection(160, 320, 1.0, is_rear=True)
    assert phi_center == pytest.approx(math.pi, rel=1e-5)

    # Right of rear camera corresponds to robot left (+y, positive bearing)
    phi_right = estimation.bearing_from_detection(240, 320, 1.0, is_rear=True)
    assert 0 < phi_right < math.pi

    # Left of rear camera corresponds to robot right (-y, negative bearing)
    phi_left = estimation.bearing_from_detection(80, 320, 1.0, is_rear=True)
    assert -math.pi < phi_left < 0
    assert abs(phi_right) == pytest.approx(abs(phi_left), rel=1e-5)

def test_estimate_package_position_rear():
    # Detection from rear camera directly behind at range 2.0m
    det = Detection("P1", "red", (140, 100, 40, 40), (160, 120), 1600, 0.9, 1600, is_rear=True)
    pose = (0.0, 0.0, 0.0)
    # Lidar with 2.0m return in all directions
    n = 360
    angles = [-math.pi + i * 2 * math.pi / n for i in range(n)]
    scan = [2.0] * n
    x, y, phi, d = estimation.estimate_package_position(det, pose, scan, angles, width=320, hfov=1.0)
    # Target should be at x = -2.0, y = 0.0
    assert x == pytest.approx(-2.0, abs=0.05)
    assert y == pytest.approx(0.0, abs=0.05)
    assert phi == pytest.approx(math.pi, abs=0.02)
    assert d == pytest.approx(2.0, abs=0.05)
