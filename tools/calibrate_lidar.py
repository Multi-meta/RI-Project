"""calibrate_lidar.py — offline LiDAR ray-order calibration + range accuracy
(Phases 4.2/4.4). Runs on logs/lidar_raw.csv written by
`DRIVE_TEST_MODE=lidar` — no simulator needed after capture.

Method:
    1. For every raw scan, compute the EXPECTED range of every ray by
       ray-marching the known static occupancy grid (the map is exactly the
       world — verified by the supervisor worldcheck) along each candidate
       ray direction.
    2. Try both ray-order conventions:
           sign=+1: ray i at angle -fov/2 + i*fov/(n-1) (index increases CCW)
           sign=-1: mirrored (index increases CW)
       plus small offset candidates. The convention with the lowest median
       |measured - expected| wins.
    3. Range-accuracy table: with the winning convention, bucket rays by
       expected distance and report the mean error per bucket.
    4. Write docs/evidence/lidar_validation.md, print the exact
       config.LIDAR_ANGLE_SIGN / _OFFSET values to set, and write
       logs/lidar_scan.csv (world-frame hits, calibrated mapping) for
       tools/plot_lidar.py -> docs/evidence/lidar_scan.png.

Run:  python tools/calibrate_lidar.py [logs/lidar_raw.csv]
"""

import csv
import math
import os
import sys

import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(ROOT, "controllers", "intellibot_controller"))

import config  # noqa: E402
from warehouse_grid import build_warehouse_grid  # noqa: E402

EV = os.path.join(ROOT, "docs", "evidence")


def load_raw(path):
    rows = list(csv.DictReader(open(path)))
    n = sum(1 for k in rows[0] if k.startswith("r"))
    t = np.array([float(r["t"]) for r in rows])
    x = np.array([float(r["x"]) for r in rows])
    y = np.array([float(r["y"]) for r in rows])
    th = np.array([float(r["theta"]) for r in rows])
    ranges = np.full((len(rows), n), np.inf)
    for i, r in enumerate(rows):
        for j in range(n):
            v = r.get(f"r{j}", "")
            if v != "":
                ranges[i, j] = float(v)
    return (t, x, y, th, ranges)


def expected_range(grid, x, y, angle, max_range, step=0.02):
    """Distance to the first occupied cell along a world-frame ray."""
    d = step
    while d <= max_range:
        px = x + d * math.cos(angle)
        py = y + d * math.sin(angle)
        r, c = grid.world_to_cell(px, py)
        if not grid.in_bounds(r, c) or grid.static[r, c]:
            return d
        d += step
    return math.inf


def calibrate(grid, x, y, th, ranges, fov):
    """Return (sign, offset, median_err) for the best convention."""
    n = ranges.shape[1]
    nominal = np.array([-fov / 2 + i * fov / (n - 1) for i in range(n)])
    best = (1.0, 0.0, math.inf)
    for sign in (1.0, -1.0):
        for offset in np.arange(-0.06, 0.061, 0.01):   # +-~3.4 deg
            errs = []
            for s in range(0, len(x), max(1, len(x) // 8)):   # subsample scans
                xi, yi, thi = x[s], y[s], th[s]
                for j in range(0, n, 4):                       # subsample rays
                    meas = ranges[s, j]
                    if not math.isfinite(meas) or meas >= config.LIDAR_MAX_RANGE:
                        continue
                    ang = thi + sign * nominal[j] + offset
                    exp = expected_range(grid, xi, yi, ang,
                                         config.LIDAR_MAX_RANGE)
                    if math.isfinite(exp):
                        errs.append(abs(meas - exp))
            med = float(np.median(errs)) if errs else math.inf
            if med < best[2]:
                best = (sign, float(offset), med)
    return best


def main(raw_path=None):
    raw_path = raw_path or os.path.join(ROOT, "logs", "lidar_raw.csv")
    if not os.path.exists(raw_path):
        print(f"no {raw_path} — run DRIVE_TEST_MODE=lidar in Webots first")
        sys.exit(1)
    t, x, y, th, ranges = load_raw(raw_path)
    n = ranges.shape[1]
    fov = 2 * math.pi
    grid = build_warehouse_grid()

    print(f"{len(t)} raw scans, {n} rays each")
    sign, offset, med = calibrate(grid, x, y, th, ranges, fov)
    print(f"best convention: LIDAR_ANGLE_SIGN={sign:+.0f}, "
          f"LIDAR_ANGLE_OFFSET={offset:.3f} (median |err| {med:.3f} m)")

    # ---- range-accuracy table with the winning convention ------------------
    nominal = np.array([-fov / 2 + i * fov / (n - 1) for i in range(n)])
    buckets = {}
    hits = []
    for s in range(len(t)):
        for j in range(n):
            meas = ranges[s, j]
            if not math.isfinite(meas) or meas >= config.LIDAR_MAX_RANGE:
                continue
            ang = th[s] + sign * nominal[j] + offset
            exp = expected_range(grid, x[s], y[s], ang, config.LIDAR_MAX_RANGE)
            if not math.isfinite(exp):
                continue
            b = min(2.0, round(exp * 2) / 2)      # 0.5 m buckets, 2.0+ lumped
            buckets.setdefault(b, []).append(meas - exp)
            if meas < config.LIDAR_MAX_RANGE - 1e-6:   # actual return = hit
                hits.append((t[s], x[s], y[s], th[s],
                             x[s] + meas * math.cos(ang),
                             y[s] + meas * math.sin(ang)))

    lines = ["# LiDAR validation (Phases 4.2/4.4)\n",
             f"Source: `{os.path.relpath(raw_path, ROOT)}` ({len(t)} scans, "
             f"{n} rays), calibrated against the known map.\n",
             f"**Ray order: LIDAR_ANGLE_SIGN = {sign:+.0f}, "
             f"LIDAR_ANGLE_OFFSET = {offset:.3f} rad** "
             f"(median |measured − expected| = {med * 100:.1f} cm)\n",
             "| expected range (m) | rays | mean err (cm) | mean |err| (cm) |",
             "|---|---|---|---|"]
    for b in sorted(buckets):
        v = np.array(buckets[b])
        lines.append(f"| {b:.1f} | {len(v)} | {v.mean() * 100:+.2f} | "
                     f"{np.abs(v).mean() * 100:.2f} |")
    lines += ["",
              "Set the values above in `config.py`, then re-run the mission;",
              "world-frame hits (calibrated) saved to `logs/lidar_scan.csv`",
              "for `tools/plot_lidar.py`."]
    os.makedirs(EV, exist_ok=True)
    out = os.path.join(EV, "lidar_validation.md")
    with open(out, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {out}")

    scan_path = os.path.join(ROOT, "logs", "lidar_scan.csv")
    with open(scan_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "theta", "hit_x", "hit_y"])
        for row in hits:
            w.writerow([round(v, 4) for v in row])
    print(f"wrote {scan_path} ({len(hits)} hits) — now run: "
          f"python tools/plot_lidar.py")

    # convenience: also render the plot directly
    try:
        import plot_lidar
        plot_lidar.main(scan_path)
    except SystemExit:
        pass


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
