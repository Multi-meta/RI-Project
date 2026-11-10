"""plot_manipulation.py — timeline of one manipulation log (checklist 17.3).

Reads a logs/manip*.csv written by intellibot_controller (or manip_check, or
tools/offline_mission_sim.py --csv) and plots joint angles, finger width,
pad force, palm range and latch state against time, shaded by FSM state.

Run:  python tools/plot_manipulation.py [logs/manip.csv] [--out docs/evidence/manip_timeline.png]
      [--title "..."]
"""

import argparse
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))


def load(path):
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit(f"{path} is empty")

    def num(v):
        try:
            return float(v)
        except (TypeError, ValueError):
            return math.nan
    cols = {k: [num(r[k]) for r in rows] for k in rows[0] if k not in ("state", "operation", "step", "arm_status")}
    cols["state"] = [r["state"] for r in rows]
    return cols


def plot(cols, out, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = cols["t"]
    fig, axes = plt.subplots(4, 1, figsize=(11, 8.5), sharex=True)
    states = cols["state"]
    spans, start = [], 0
    for i in range(1, len(states) + 1):
        if i == len(states) or states[i] != states[start]:
            spans.append((t[start], t[i - 1], states[start]))
            start = i
    palette = ["#eef2ff", "#fef3c7", "#ecfccb", "#fce7f3", "#e0f2fe", "#f5f5f4"]
    for ax in axes:
        for k, (a, b, s) in enumerate(spans):
            ax.axvspan(a, b, color=palette[k % len(palette)], lw=0)
    for k, (a, b, s) in enumerate(spans):
        if b - a > 0.03 * (t[-1] - t[0]):
            axes[0].text((a + b) / 2, 1.02, s, transform=axes[0].get_xaxis_transform(),
                         ha="center", fontsize=7, rotation=30)
    for name in ("q_shoulder", "q_elbow", "q_wrist"):
        axes[0].plot(t, [math.degrees(v) for v in cols[name]], label=name[2:])
    axes[0].set_ylabel("joint (deg)")
    axes[0].legend(fontsize=8, loc="upper right")
    axes[1].plot(t, [v * 100 for v in cols["finger_width"]], color="#dc2626", label="finger width")
    axes[1].plot(t, [v * 100 for v in cols["palm_range"]], color="#2563eb", label="palm range")
    axes[1].set_ylabel("cm")
    axes[1].legend(fontsize=8, loc="upper right")
    axes[2].plot(t, cols["force_left"], label="left pad")
    axes[2].plot(t, cols["force_right"], label="right pad", ls="--")
    axes[2].set_ylabel("force (N)")
    axes[2].legend(fontsize=8, loc="upper right")
    axes[3].step(t, cols["latch_present"], label="latch peer present", where="post")
    axes[3].step(t, cols["latched"], label="latch locked", where="post", ls="--")
    axes[3].set_ylabel("latch")
    axes[3].set_xlabel("simulation time (s)")
    axes[3].legend(fontsize=8, loc="upper right")
    fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=120)
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?", default=os.path.join(ROOT, "logs", "manip.csv"))
    ap.add_argument("--out", default=os.path.join(ROOT, "docs", "evidence", "manip_timeline.png"))
    ap.add_argument("--title", default="Manipulation timeline (Webots run)")
    a = ap.parse_args()
    plot(load(a.log), a.out, a.title)


if __name__ == "__main__":
    sys.exit(main())
