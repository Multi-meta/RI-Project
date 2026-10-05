"""Detector tests on synthetic frames (Webots frames validated separately)."""

import cv2
import numpy as np
import pytest

import detector
import config


def make_scene(color_bgr, center=(160, 120), size=60, bg=(128, 128, 128)):
    img = np.full((240, 320, 3), bg, dtype=np.uint8)
    x, y = center
    h = size // 2
    cv2.rectangle(img, (x - h, y - h), (x + h, y + h), color_bgr, -1)
    return img


def test_detects_blue_package():
    img = make_scene((255, 0, 0))                 # pure blue in BGR
    dets = detector.detect_colors(img)
    blues = [d for d in dets if d.color == "blue"]
    assert len(blues) == 1
    d = blues[0]
    assert d.pkg_id == "P3"
    u, v = d.centroid
    assert abs(u - 160) < 3 and abs(v - 120) < 3
    assert 50 <= d.bbox[2] <= 70                  # bbox width ~ size


def test_detects_red_and_green():
    img = np.full((240, 320, 3), (128, 128, 128), dtype=np.uint8)
    cv2.rectangle(img, (20, 20), (80, 80), (0, 0, 255), -1)     # red
    cv2.rectangle(img, (200, 120), (280, 200), (0, 255, 0), -1) # green
    dets = detector.detect_colors(img)
    colors = {d.color for d in dets}
    assert colors == {"red", "green"}


def test_zone_colors_not_confused():
    """Yellow (zone A) and cyan (zone B) must NOT trigger package colors."""
    img = np.full((240, 320, 3), (128, 128, 128), dtype=np.uint8)
    cv2.rectangle(img, (10, 10), (110, 110), (0, 255, 255), -1)  # yellow
    cv2.rectangle(img, (200, 100), (310, 220), (255, 255, 0), -1)  # cyan
    dets = detector.detect_colors(img)
    assert dets == [], f"zone colors misdetected as {dets}"


def test_small_blob_rejected():
    img = make_scene((255, 0, 0), size=12)        # tiny -> below MIN_AREA
    assert detector.detect_package(img, "P3") is None


def test_annotate_draws():
    img = make_scene((255, 0, 0))
    dets = detector.detect_colors(img)
    out = detector.annotate(img, dets)
    assert out.shape == img.shape
    assert not np.array_equal(out, img)           # something was drawn


def test_detect_package_specific():
    img = make_scene((0, 255, 0))                 # green
    assert detector.detect_package(img, "P2") is not None
    assert detector.detect_package(img, "P3") is None
