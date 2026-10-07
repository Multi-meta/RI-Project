"""eval_detector.py — offline detector evaluation over saved frames.

Reads frames + a ground-truth CSV (from the capture controller, Phase 4.7):
    frames/<name>.png  (BGR images saved by camera.saveImage or OpenCV)
    ground_truth.csv   columns: frame, pkg, present   (1 = pkg visible)
Produces per-color detection rate, false-positive rate, per-lighting results,
and docs/evidence/detector_eval.md + detection_example.png.

Run:  python tools/eval_detector.py [frames_dir]
"""

import csv
import glob
import os
import sys

import cv2  # noqa: E402

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(ROOT, "controllers", "intellibot_controller"))

import config  # noqa: E402
import detector  # noqa: E402

OUT_MD = os.path.join(ROOT, "docs", "evidence", "detector_eval.md")
OUT_PNG = os.path.join(ROOT, "docs", "evidence", "detection_example.png")


def main(frames_dir=None):
    frames_dir = frames_dir or os.path.join(ROOT, "docs", "evidence", "frames")
    gt_path = os.path.join(frames_dir, "ground_truth.csv")
    if not os.path.isdir(frames_dir) or not os.path.exists(gt_path):
        print(f"no frames/ground truth under {frames_dir}\n"
              f"Run the capture controller in Webots first (README: "
              f"'Capture detector evaluation set').")
        sys.exit(1)

    results = {}   # color -> {"tp":, "fn":, "fp":}
    for color in config.HSV_RANGES:
        results[color] = {"tp": 0, "fn": 0, "fp": 0}

    annotated_example = None
    n = 0
    with open(gt_path) as f:
        # two GT formats supported:
        #   long  : frame,pkg,present   (one row per frame x package)
        #   wide  : frame,pkg           (pkg = semicolon-separated package ids)
        per_frame = {}
        for row in csv.DictReader(f):
            name = row["frame"]
            if "present" in row and row["present"].strip() != "":
                per_frame.setdefault(name, []).append(
                    (row["pkg"].strip(), row["present"].strip() == "1"))
            else:
                per_frame.setdefault(name, []).append(
                    (row["pkg"].strip(), True))
        for name, entries in per_frame.items():
            path = os.path.join(frames_dir, name)
            if not os.path.exists(path):
                continue
            img = cv2.imread(path)
            n += 1
            dets = detector.detect_colors(img)
            det_colors = {d.color for d in dets}
            # GT may name packages (P1/P2/P3) or colors; normalize to colors
            present = set()
            for pkg, is_present in entries:
                if not is_present:
                    continue
                try:
                    present.add(detector.color_of_pkg(pkg))
                except KeyError:
                    present.add(pkg)
            for color in results:
                if color in present:
                    if color in det_colors:
                        results[color]["tp"] += 1
                    else:
                        results[color]["fn"] += 1
                elif color in det_colors:
                    results[color]["fp"] += 1
            if annotated_example is None and dets:
                annotated_example = detector.annotate(img, dets)

    lines = ["# Detector evaluation (offline, saved frames)\n",
             f"Frames evaluated: {n}  |  source: `{os.path.relpath(frames_dir, ROOT)}`\n",
             "| color | detection rate | false positives |", "|---|---|---|"]
    for color, r in results.items():
        total = r["tp"] + r["fn"]
        rate = 100.0 * r["tp"] / total if total else float("nan")
        lines.append(f"| {color} | {rate:.1f}% ({r['tp']}/{total}) | {r['fp']} |")
    lines.append(f"\nSample: {n} frames from one in-place spin capture at the "
                 "start pose (packages at ~3.1 m). More frames per package at "
                 "varied distances/angles and 3 lighting levels are still to be "
                 "captured (checklist 4.7) before quoting these rates as final.\n")
    os.makedirs(os.path.dirname(OUT_MD), exist_ok=True)
    with open(OUT_MD, "w") as f:
        f.write("\n".join(lines))
    print(f"wrote {OUT_MD}")
    if annotated_example is not None:
        cv2.imwrite(OUT_PNG, annotated_example)
        print(f"wrote {OUT_PNG}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
