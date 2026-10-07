"""estimation.py — package world-position estimation from camera + LiDAR.

Pure functions (no Webots) implementing the spatial transformation chain of
project_summary.md §10.8:

    pixel column u -> bearing phi (robot frame) -> range d (LiDAR at that
    bearing) -> world position via pose + polar transform.

Fallback range (no LiDAR return on the package): pinhole model
    d = f_px * PKG_SIZE / bbox_width_px
"""

import math

import controller_output.intellibot_controller.config as config


def focal_px(width, hfov):
    """Horizontal focal length in pixels for a pinhole camera."""
    return (width / 2.0) / math.tan(hfov / 2.0)


def bearing_from_detection(centroid_u, width, hfov):
    """Image column (px) -> bearing in the robot frame (rad, left = +).

    Image x grows to the RIGHT, robot-frame bearings grow to the LEFT,
    hence the minus sign.
    """
    f = focal_px(width, hfov)
    return -math.atan2(centroid_u - width / 2.0, f)


def range_at_bearing(scan, angles, phi, window=None):
    """LiDAR range at bearing phi: min over +-window. inf if no valid ray."""
    if window is None:
        window = config.PKG_BEARING_WINDOW
    from controller_output.intellibot_controller.mapping import obstacle_in_sector
    return obstacle_in_sector(scan, angles, phi - window, phi + window)


def fallback_range(bbox_w_px, width, hfov, pkg_size=None):
    """Pinhole depth estimate from the bbox width in pixels."""
    if pkg_size is None:
        pkg_size = config.PKG_SIZE
    if bbox_w_px <= 0:
        return math.inf
    return focal_px(width, hfov) * pkg_size / bbox_w_px


def estimate_package_position(detection, pose, scan=None, angles=None,
                              width=None, hfov=None):
    """Detection + robot pose (+ LiDAR) -> (x, y) world position estimate.

    pose: (x, y, theta). Uses the LiDAR range at the detection's bearing when
    available, otherwise the pinhole fallback.
    """
    if width is None:
        width = config.CAMERA_WIDTH
    if hfov is None:
        hfov = config.CAMERA_FOV
    x, y, theta = pose
    phi = bearing_from_detection(detection.centroid[0], width, hfov)
    d = math.inf
    if scan is not None and angles is not None:
        d = range_at_bearing(scan, angles, phi)
    if not math.isfinite(d) or d <= 0:
        bx, by, bw, bh = detection.bbox
        d = fallback_range(bw, width, hfov)
    return x + d * math.cos(theta + phi), y + d * math.sin(theta + phi), phi, d
