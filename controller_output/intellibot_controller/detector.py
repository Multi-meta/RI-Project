"""detector.py — pure-OpenCV HSV package detector (no Webots).

Pipeline (project_summary.md §10.8):
    BGR -> HSV -> per-color masks (red needs two hue ranges) ->
    morphological open+close -> contours -> largest blob with
    area >= MIN_AREA and plausible aspect ratio -> Detection.

`score` is a DOCUMENTED HEURISTIC (contour-area / bbox-area solidity x size
factor), NOT a statistical confidence. Say "heuristic score" on slides.
"""

import cv2
import numpy as np

import controller_output.intellibot_controller.config as config


class Detection:
    """One detected package in one camera frame."""

    def __init__(self, pkg_id, color, bbox, centroid, area, score, mask_area):
        self.pkg_id = pkg_id        # semantic id ("P1"/"P2"/"P3") via color
        self.color = color          # "red" / "green" / "blue"
        self.bbox = bbox            # (x, y, w, h) top-left pixel coords
        self.centroid = centroid    # (u, v) pixel coords of blob center
        self.area = area            # contour area, px^2
        self.score = score          # heuristic 0..1
        self.mask_area = mask_area  # pixels in the color mask

    def __repr__(self):
        return (f"Detection({self.pkg_id}, {self.color}, bbox={self.bbox}, "
                f"centroid={self.centroid}, area={self.area:.0f}, "
                f"score={self.score:.2f})")


def color_of_pkg(pkg_id):
    return config.PACKAGE_COLORS[pkg_id]


def pkg_of_color(color):
    for pid, c in config.PACKAGE_COLORS.items():
        if c == color:
            return pid
    return None


def detect_colors(bgr, colors=None, hsv_ranges=None):
    """Detect colored package cubes in a BGR ndarray.

    Returns list[Detection], best (largest valid) blob per requested color.
    """
    if hsv_ranges is None:
        hsv_ranges = config.HSV_RANGES
    if colors is None:
        colors = list(hsv_ranges.keys())

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    detections = []
    for color in colors:
        ranges = hsv_ranges.get(color)
        if not ranges:
            continue
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        # clean the mask: open removes speckle, close fills pinholes
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < config.MIN_CONTOUR_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if w == 0 or h == 0:
                continue
            aspect = w / float(h)
            if not (config.ASPECT_MIN <= aspect <= config.ASPECT_MAX):
                continue
            if best is None or area > best[0]:
                best = (area, (x, y, w, h), cnt)
        if best is None:
            continue
        area, bbox, cnt = best
        bx, by, bw, bh = bbox
        solidity = area / float(bw * bh)                 # how box-like
        size_factor = min(1.0, area / 4000.0)            # closer = larger px area
        score = float(max(0.0, min(1.0, solidity * size_factor)))
        m = cv2.moments(cnt)
        cx = m["m10"] / m["m00"] if m["m00"] else bx + bw / 2.0
        cy = m["m01"] / m["m00"] if m["m00"] else by + bh / 2.0
        detections.append(Detection(pkg_of_color(color), color, bbox,
                                    (cx, cy), area, score, int(mask.sum())))
    return detections


def detect_package(bgr, pkg_id, hsv_ranges=None):
    """Detect one specific package by its color. Returns Detection or None."""
    color = color_of_pkg(pkg_id)
    dets = detect_colors(bgr, colors=[color], hsv_ranges=hsv_ranges)
    return dets[0] if dets else None


def annotate(bgr, detections):
    """Return a copy of the image with bboxes + labels drawn (for evidence)."""
    img = bgr.copy()
    for d in detections:
        x, y, w, h = d.bbox
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        label = f"{d.pkg_id} {d.score:.2f}"
        cv2.putText(img, label, (x, max(12, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
    return img
