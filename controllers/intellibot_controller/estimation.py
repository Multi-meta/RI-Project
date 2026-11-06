"""estimation.py — package world-position estimation from camera + LiDAR.

Pure functions (no Webots) implementing the spatial transformation chain of
project_summary.md §10.8:

    pixel column u -> bearing phi (robot frame) -> range d (LiDAR at that
    bearing) -> world position via pose + polar transform.

Fallback range (no LiDAR return on the package): pinhole or spherical model
    d = f_px * PKG_SIZE / bbox_width_px
"""

import math

import config


def focal_px(width, hfov):
    """Horizontal focal length in pixels for a pinhole camera."""
    hfov_c = min(float(hfov), math.pi - 1e-3)
    return (width / 2.0) / math.tan(hfov_c / 2.0)


def bearing_from_detection(centroid_u, width, hfov, is_rear=False):
    """Image column (px) -> bearing in the robot frame (rad, left = positive)."""
    from mapping import wrap_angle
    if hfov >= 2.0:
        angle_offset = (centroid_u - width / 2.0) * (hfov / width)
        if is_rear:
            return wrap_angle(math.pi - angle_offset)
        return wrap_angle(-angle_offset)

    f = focal_px(width, hfov)
    if is_rear:
        phi_rear = math.atan2(centroid_u - width / 2.0, f)
        return wrap_angle(math.pi - phi_rear)
    return -math.atan2(centroid_u - width / 2.0, f)


def range_at_bearing(scan, angles, phi, window=None):
    """LiDAR range at bearing phi: min over +-window. inf if no valid ray."""
    if window is None:
        window = config.PKG_BEARING_WINDOW
    from mapping import obstacle_in_sector
    return obstacle_in_sector(scan, angles, phi - window, phi + window)


def fallback_range(bbox_w_px, width, hfov, pkg_size=None):
    """Depth estimate from the bbox width in pixels."""
    if pkg_size is None:
        pkg_size = config.PKG_SIZE
    if bbox_w_px <= 0:
        return math.inf
    if hfov >= 2.0:
        ang_w = max(1e-4, bbox_w_px * (hfov / width))
        return pkg_size / ang_w
    return focal_px(width, hfov) * pkg_size / bbox_w_px


def estimate_package_position(detection, pose, scan=None, angles=None,
                              width=None, hfov=None):
    """Detection + robot pose (+ LiDAR) -> (x, y) world position estimate.

    pose: (x, y, theta). Uses the LiDAR range at the detection's bearing when
    available, otherwise the fallback model.
    """
    if width is None:
        width = config.CAMERA_WIDTH
    if hfov is None:
        hfov = config.CAMERA_FOV
    x, y, theta = pose
    is_rear = getattr(detection, "is_rear", False)
    phi = bearing_from_detection(detection.centroid[0], width, hfov, is_rear=is_rear)
    d = math.inf
    if scan is not None and angles is not None:
        d = range_at_bearing(scan, angles, phi)
    if not math.isfinite(d) or d <= 0:
        bx, by, bw, bh = detection.bbox
        d = fallback_range(bw, width, hfov)
    return x + d * math.cos(theta + phi), y + d * math.sin(theta + phi), phi, d
