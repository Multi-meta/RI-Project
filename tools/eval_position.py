"""eval_position.py — package world-position estimate error (checklist 4.9).

Uses the captured spin frames: for every frame with detections, runs the
§10.8 estimation chain (pixel -> bearing -> world) with the PINHOLE fallback
range (no per-frame LiDAR logs exist), and compares against the true package
positions from config. The result therefore measures the fallback path; the
LiDAR-fused estimate is evaluated live (position error printed by the
controller during FIND_PACKAGE).

Writes docs/evidence/position_estimate.md.

Run:  python tools/eval_position.py
"""

import math
import os
import sys

import cv2  # noqa: E402

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(ROOT, "controllers", "intellibot_controller"))

import config  # noqa: E402
import detector  # noqa: E402
import estimation  # noqa: E402

EV = os.path.join(ROOT, "docs", "evidence")
OUT = os.path.join(EV, "position_estimate.md")


def main():
    frames = os.path.join(EV, "frames")
    true_pos = {pid.replace("PKG_", ""): (x, y)
                for pid, x, y, _c in config.PACKAGES}
    pose0 = config.ROBOT_START

    rows, errs = [], []
    for name in sorted(os.listdir(frames)):
        if not name.endswith(".png"):
            continue
        yaw_deg = name.split("_")[-1].split(".")[0]
        pose = (pose0[0], pose0[1], math.radians(float(yaw_deg)))
        img = cv2.imread(os.path.join(frames, name))
        for det in detector.detect_colors(img):
            ex, ey, phi, d = estimation.estimate_package_position(
                det, pose, None, None)
            tx, ty = true_pos[det.pkg_id]
            err = math.hypot(ex - tx, ey - ty)
            errs.append(err)
            rows.append((name, det.pkg_id, d, math.degrees(phi), ex, ey, err))

    lines = ["# Package position estimate (checklist 4.9)\n",
             "Offline evaluation of the §10.8 chain on the spin frames with the",
             "PINHOLE fallback range (d = f_px·PKG_SIZE/bbox_width) — no per-frame",
             "LiDAR logs exist for these frames. The LiDAR-fused estimate is",
             "printed by the controller during FIND_PACKAGE (live number).\n",
             "| frame | pkg | range est (m) | bearing (deg) | est (x, y) | error (m) |",
             "|---|---|---|---|---|---|"]
    for name, pid, d, phi, ex, ey, err in rows:
        lines.append(f"| {name} | {pid} | {d:.2f} | {phi:+.1f} | "
                     f"({ex:.2f}, {ey:.2f}) | {err:.2f} |")
    if errs:
        lines += ["", f"- mean error: **{sum(errs)/len(errs):.2f} m**",
                  f"- max error: **{max(errs):.2f} m**",
                  "- note: at ~3.1 m the pinhole range is quantization-limited "
                  "(±1 px bbox ≈ ±5 % range). The live LiDAR-fused estimate is "
                  "expected to be clearly better; record it from the console "
                  "during the demo run."]
    os.makedirs(EV, exist_ok=True)
    with open(OUT, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {OUT} ({len(errs)} estimates)")


if __name__ == "__main__":
    main()
