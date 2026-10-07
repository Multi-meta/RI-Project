"""build_frame_gt.py — derive ground_truth.csv for captured spin frames.

Frames named <prefix>_<yaw_deg>.png are assumed captured at the robot start
pose (config.ROBOT_START) with yaw = <yaw_deg>. A package is "present" in a
frame when its bearing from the camera lies inside the horizontal FOV and
the straight line to it is not blocked by a static obstacle (checked against
the known map: no occupied cell on the segment, sampled every 5 cm).

Writes ground_truth.csv next to the frames: columns frame,pkg,present.
Run:  python tools/build_frame_gt.py [frames_dir]
"""

import math
import os
import re
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(ROOT, "controllers", "intellibot_controller"))

import config  # noqa: E402
from mapping import bresenham  # noqa: E402
from warehouse_grid import build_warehouse_grid  # noqa: E402


def visible_from(pose, target, grid):
    """Package visible if inside FOV and LOS not blocked by static obstacles."""
    dx, dy = target[0] - pose[0], target[1] - pose[1]
    dist = math.hypot(dx, dy)
    bearing = math.atan2(dy, dx)
    err = abs(math.atan2(math.sin(bearing - pose[2]),
                         math.cos(bearing - pose[2])))
    if err > config.CAMERA_FOV / 2.0:
        return False
    # sample the segment every ~5 cm against the static map
    n = int(dist / 0.05)
    for i in range(1, n):
        x = pose[0] + dx * i / n
        y = pose[1] + dy * i / n
        r, c = grid.world_to_cell(x, y)
        if grid.static[r, c]:
            return False
    return True


def main(frames_dir=None):
    frames_dir = frames_dir or os.path.join(ROOT, "docs", "evidence", "frames")
    grid = build_warehouse_grid()
    pose0 = config.ROBOT_START
    pkg_pos = {pid: (x, y) for pid, x, y, _c in config.PACKAGES}

    rows = []
    for name in sorted(os.listdir(frames_dir)):
        m = re.match(r".*?(\d+)\.png$", name)
        if not m:
            continue
        yaw = math.radians(int(m.group(1)))
        pose = (pose0[0], pose0[1], yaw)
        present = [pid for pid, xy in pkg_pos.items()
                   if visible_from(pose, xy, grid)]
        rows.append((name, present))
        print(f"{name}: {present or 'none'}")

    out = os.path.join(frames_dir, "ground_truth.csv")
    with open(out, "w") as f:
        f.write("frame,pkg,present\n")
        for name, present in rows:
            for pid in ("P1", "P2", "P3"):
                f.write(f"{name},{pid},{1 if f'PKG_{pid}' in present else 0}\n")
    print(f"wrote {out}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
