"""Front/rear camera pipeline: conversion, orientation, separation, health."""

import math
import re

import numpy as np
import pytest

import config
import detector
import estimation
from camera_pipeline import (FRONT, REAR, CameraFrameError, FrameHealthMonitor,
                             bgra_to_bgr, make_frame)


def bgra(img_bgr, alpha=255):
    h, w, _ = img_bgr.shape
    a = np.full((h, w, 1), alpha, np.uint8)
    return np.concatenate([img_bgr, a], axis=2).tobytes()


# ------------------------------------------------------------- conversion
def test_bgra_to_bgr_keeps_channel_order_and_layout():
    img = np.zeros((2, 3, 3), np.uint8)
    img[0, 0] = (10, 20, 30)          # B, G, R
    img[1, 2] = (200, 100, 50)
    out = bgra_to_bgr(bgra(img, alpha=7), 3, 2)
    assert out.shape == (2, 3, 3) and out.dtype == np.uint8
    assert tuple(out[0, 0]) == (10, 20, 30)     # no B<->R swap
    assert tuple(out[1, 2]) == (200, 100, 50)   # no vertical/horizontal flip
    assert out.flags["C_CONTIGUOUS"]


def test_bgra_buffer_is_copied_not_aliased():
    buf = bytearray(bgra(np.full((2, 2, 3), 9, np.uint8)))
    out = bgra_to_bgr(bytes(buf), 2, 2)
    buf[0] = 0
    assert out[0, 0, 0] == 9


@pytest.mark.parametrize("buf,w,h", [(None, 320, 240), (b"\x00" * 10, 320, 240),
                                     (b"\x00" * 16, 0, 4)])
def test_invalid_buffers_raise(buf, w, h):
    with pytest.raises(CameraFrameError):
        bgra_to_bgr(buf, w, h)


def test_converted_frame_works_with_the_existing_detector():
    """A pure-blue square on the RIGHT must be detected as P3 on the right."""
    img = np.full((240, 320, 3), 128, np.uint8)
    img[100:140, 220:260] = (255, 0, 0)          # BGR blue
    frame = bgra_to_bgr(bgra(img), 320, 240)
    dets = detector.detect_colors(frame)
    assert [d.pkg_id for d in dets] == ["P3"]
    assert dets[0].centroid[0] > 160


# ---------------------------------------------------------------- health
def test_make_frame_flags_invalid_blank_and_frozen():
    f = make_frame(FRONT, None, 320, 240, 0.0)
    assert not f.valid and f.health == "INVALID" and f.image is None
    mon = FrameHealthMonitor(FRONT, frozen_frames=3)
    flat = bgra(np.full((24, 32, 3), 40, np.uint8))
    assert make_frame(FRONT, flat, 32, 24, 0.0, mon).health == "BLANK"
    rng = np.random.default_rng(0)
    tex = bgra(rng.integers(0, 255, (24, 32, 3), dtype=np.uint8))
    healths = [make_frame(FRONT, tex, 32, 24, i * 0.1, mon).health for i in range(5)]
    assert healths[0] == "OK" and healths[-1] == "FROZEN"
    other = bgra(rng.integers(0, 255, (24, 32, 3), dtype=np.uint8))
    assert make_frame(FRONT, other, 32, 24, 1.0, mon).health == "OK"


# ------------------------------------------------- orientation / bearings
def _project(point, cam_x, yaw, fov=config.CAMERA_FOV, width=config.CAMERA_WIDTH):
    """Pinhole projection of a robot-frame point into a camera looking along
    its +x after a yaw about z (Webots camera convention). Returns column u."""
    dx, dy = point[0] - cam_x, point[1]
    xc = math.cos(yaw) * dx + math.sin(yaw) * dy      # camera forward
    yc = -math.sin(yaw) * dx + math.cos(yaw) * dy     # camera left
    f = (width / 2) / math.tan(fov / 2)
    return width / 2 - f * yc / xc


@pytest.mark.parametrize("pt", [(1.0, 0.3), (1.5, -0.4), (0.8, 0.0)])
def test_front_bearing_matches_projection(pt):
    u = _project(pt, config.CAMERA_MOUNT_X, 0.0)
    phi = estimation.bearing_from_detection(u, config.CAMERA_WIDTH, config.CAMERA_FOV)
    assert phi == pytest.approx(math.atan2(pt[1], pt[0] - config.CAMERA_MOUNT_X), abs=1e-9)


@pytest.mark.parametrize("pt", [(-1.0, 0.3), (-1.5, -0.4), (-0.9, 0.0)])
def test_rear_bearing_matches_projection_and_is_not_mirrored(pt):
    u = _project(pt, config.REAR_CAMERA_MOUNT_X, config.REAR_CAMERA_YAW)
    if pt[1] > 0:
        assert u > config.CAMERA_WIDTH / 2      # robot-LEFT appears on image RIGHT
    phi = estimation.bearing_from_detection(u, config.CAMERA_WIDTH,
                                            config.REAR_CAMERA_FOV, is_rear=True)
    true = math.atan2(pt[1], pt[0] - config.REAR_CAMERA_MOUNT_X)
    assert math.remainder(phi - true, 2 * math.pi) == pytest.approx(0.0, abs=1e-9)


def test_planar_fov_is_valid_for_both_cameras():
    for fov in (config.CAMERA_FOV, config.REAR_CAMERA_FOV):
        assert 0.0 < fov < math.pi
    assert config.CAMERA_PROJECTION == "planar"


# ----------------------------------------------- perception: no interchange
class FakeDev:
    def __init__(self, name, image=None, fov=1.0, w=32, h=24):
        self.name, self.image, self.fov, self.w, self.h = name, image, fov, w, h
        self.enabled = False

    def enable(self, ts): self.enabled = True
    def getWidth(self): return self.w
    def getHeight(self): return self.h
    def getFov(self): return self.fov
    def getImage(self): return self.image
    def getHorizontalResolution(self): return 360
    def getMaxRange(self): return 4.0
    def getRangeImage(self): return [1.0] * 360
    def setPosition(self, p): pass
    def setVelocity(self, v): pass
    def getValue(self): return 0.0
    def getValues(self): return [0.0, 0.0, 0.0]
    def getRollPitchYaw(self): return [0.0, 0.0, 0.0]


class FakeRobot:
    def __init__(self, devices):
        self.devices = devices

    def getDevice(self, name):
        return self.devices.get(name) or FakeDev(name)


def _square(color_bgr, right):
    img = np.full((24, 32, 3), 128, np.uint8)
    x0 = 20 if right else 2
    img[6:18, x0:x0 + 12] = color_bgr
    return bgra(img)


def make_perception(front_img, rear_img, rear_fov=1.0):
    import perception
    lidar = FakeDev(config.LIDAR_NAME)
    lidar.getFov = lambda: 2 * math.pi
    devs = {config.CAMERA_NAME: FakeDev(config.CAMERA_NAME, front_img),
            config.REAR_CAMERA_NAME: FakeDev(config.REAR_CAMERA_NAME, rear_img, rear_fov),
            config.LIDAR_NAME: lidar}
    return perception.Perception(FakeRobot(devs), 16), devs


def test_front_and_rear_frames_come_from_their_own_devices():
    front = _square((255, 0, 0), right=True)      # blue on the right
    rear = _square((0, 0, 255), right=False)      # red on the left
    p, devs = make_perception(front, rear)
    frames = p.get_camera_frames(1.0)
    assert frames[FRONT].source == FRONT and frames[REAR].source == REAR
    assert tuple(frames[FRONT].image[10, 25]) == (255, 0, 0)
    assert tuple(frames[REAR].image[10, 5]) == (0, 0, 255)
    assert devs[config.CAMERA_NAME].enabled and devs[config.REAR_CAMERA_NAME].enabled
    dets = p.get_detections(frames[FRONT].image, frames[REAR].image)
    by_id = {d.pkg_id: d for d in dets}
    assert by_id["P3"].is_rear is False and by_id["P1"].is_rear is True


def test_rear_detection_uses_rear_camera_parameters():
    p, devs = make_perception(_square((255, 0, 0), True), _square((0, 0, 255), True),
                              rear_fov=0.8)

    class D:
        centroid, bbox, is_rear = (16, 12), (10, 6, 12, 12), True
    assert p.camera_for(D()) is p.rear_cam
    _x, _y, phi, _d = p.estimate_package_position(D(), (0, 0, 0), None, None)
    assert abs(math.remainder(phi - math.pi, 2 * math.pi)) < 1e-9   # centre -> straight back


def test_missing_frames_do_not_crash_detection():
    p, _ = make_perception(None, None)
    frames = p.get_camera_frames(0.0)
    assert not frames[FRONT].valid and not frames[REAR].valid
    assert p.get_detections(frames[FRONT].image, frames[REAR].image) == []


def test_invalid_fov_is_reported(capsys):
    make_perception(None, None, rear_fov=math.pi)
    assert "not a valid planar FOV" in capsys.readouterr().out


def test_lidar_self_filter_removes_only_close_returns():
    from mapping import self_filter
    r = self_filter([0.11, 0.2, np.nan, -1.0, 3.0], config.LIDAR_SELF_FILTER_RANGE)
    assert np.isinf(r[0]) and r[1] == 0.2 and np.isinf(r[2]) and np.isinf(r[3])


# --------------------------------------------- generated world <-> config
def test_generated_cameras_match_config():
    import generate_world
    text = generate_world.build_world("intellibot_controller")
    cams = re.findall(r"Camera \{\s*translation ([^\n]+)\n(?:\s*rotation ([^\n]+)\n)?"
                      r".*?name \"(\w+)\"\s*fieldOfView ([\d.]+)", text, re.S)
    by_name = {c[2]: c for c in cams}
    fx, fy, fz = map(float, by_name[config.CAMERA_NAME][0].split())
    assert (fx, fz) == pytest.approx((config.CAMERA_MOUNT_X, config.CAMERA_MOUNT_Z))
    assert by_name[config.CAMERA_NAME][1] == ""                      # not rotated
    rx, ry, rz = map(float, by_name[config.REAR_CAMERA_NAME][0].split())
    assert rx == pytest.approx(config.REAR_CAMERA_MOUNT_X)
    axis_angle = list(map(float, by_name[config.REAR_CAMERA_NAME][1].split()))
    assert axis_angle[:3] == [0, 0, 1] and axis_angle[3] == pytest.approx(math.pi, abs=1e-5)
    for c in cams:
        assert 0 < float(c[3]) < math.pi
    # housings sit BEHIND each lens (negative x in the camera's own frame)
    assert text.count("Pose { translation -0.008 0 0 children [ Shape") == 2
