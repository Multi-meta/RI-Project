"""plot_lidar.py — polar/XY plot of a logged LiDAR scan (offline part).

The in-sim capture step (a controller writing logs/lidar_scan.csv with world
points + the robot pose) feeds this script. Also renders world-frame hit
points overlaid on the known map -> docs/evidence/lidar_scan.png.

Run:  python tools/plot_lidar.py [logs/lidar_scan.csv]
CSV columns: t, x, y, theta, hit_x, hit_y  (one row per hit; t/pose repeated)
"""

import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(ROOT, "controllers", "intellibot_controller"))

import config  # noqa: E402
from warehouse_grid import build_warehouse_grid  # noqa: E402

OUT = os.path.join(ROOT, "docs", "evidence", "lidar_scan.png")


def main(csv_path=None):
    csv_path = csv_path or os.path.join(ROOT, "logs", "lidar_scan.csv")
    hits, pose = [], None
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            pose = (float(row["x"]), float(row["y"]), float(row["theta"]))
            hits.append((float(row["hit_x"]), float(row["hit_y"])))
    if pose is None:
        print(f"no rows in {csv_path} — run the capture controller first")
        sys.exit(1)

    grid = build_warehouse_grid()
    fig, ax = plt.subplots(figsize=(10, 7.5))
    occ = grid.static
    img = np.ma.masked_where(~occ, occ)
    ax.imshow(img, cmap="Greys", origin="lower",
              extent=[grid.x_min, grid.x_max, grid.y_min, grid.y_max], alpha=0.8)
    if hits:
        hx, hy = zip(*hits)
        ax.plot(hx, hy, ".", color="tab:red", ms=2, label="LiDAR hits")
    ax.plot(pose[0], pose[1], "s", color="tab:green", ms=10, label="robot")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)
    ax.legend()
    ax.set_title(f"LiDAR scan in world frame ({len(hits)} hits)")
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, dpi=150)
    print(f"saved {OUT}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
