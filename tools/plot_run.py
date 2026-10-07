"""plot_run.py — planned vs actual overlay from a mission run log (6.3).

Reads logs/run.csv (or logs/run_<i>.csv), re-plans the A* reference route
from the first logged pose to Zone B on the known map, and plots:
    * known obstacles + inflation
    * A* reference path (dashed)
    * the robot's ACTUAL trajectory, colored by FSM state
    * Zone B goal, package P3, start marker
-> docs/evidence/planned_vs_actual.png

Run:  python tools/plot_run.py [logs/run.csv]
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
from path_utils import plan_to  # noqa: E402
from warehouse_grid import build_warehouse_grid  # noqa: E402

OUT = os.path.join(ROOT, "docs", "evidence", "planned_vs_actual.png")

STATE_COLORS = {
    "FIND_PACKAGE": "#1d4ed8",
    "NAVIGATE": "#ea580c",
    "PICK": "#f59e0b",
    "PLAN_DELIVERY": "#ca8a04",
    "DELIVER": "#0d9488",
    "DONE": "#16a34a",
    "ERROR": "#dc2626",
}


def plot_run(csv_path, out_path=OUT):
    rows = list(csv.DictReader(open(csv_path)))
    xs = [float(r["x"]) for r in rows]
    ys = [float(r["y"]) for r in rows]
    states = [r["state"] for r in rows]
    cte = [float(r["cross_track_err"]) for r in rows if r["cross_track_err"]]

    grid = build_warehouse_grid()
    occ = grid.inflate(config.INFLATION_RADIUS)

    # A* reference route from the first pose to Zone B (the mission's second
    # leg goal; the first leg goal depends on the live package estimate)
    start = (xs[0], ys[0])
    ref, info = plan_to(grid, start, config.ZONE_B_CENTER,
                        resample_spacing=0.0,
                        inflation_radius=config.INFLATION_RADIUS)

    fig, ax = plt.subplots(figsize=(10, 7.5))
    img = np.ma.masked_where(~occ, occ)
    ax.imshow(img, cmap="Oranges", origin="lower",
              extent=[grid.x_min, grid.x_max, grid.y_min, grid.y_max],
              alpha=0.55, vmin=0, vmax=3)
    st = np.ma.masked_where(~grid.static, grid.static)
    ax.imshow(st, cmap="Greys", origin="lower",
              extent=[grid.x_min, grid.x_max, grid.y_min, grid.y_max],
              alpha=0.9, vmin=0, vmax=3)

    if ref is not None:
        rx = [p[0] for p in ref]
        ry = [p[1] for p in ref]
        ax.plot(rx, ry, "--", color="tab:blue", lw=2,
                label=f"A* reference ({info['length_m']:.2f} m)")

    # actual trajectory, one segment per state color
    for state in dict.fromkeys(states):
        seg = [(float(r["x"]), float(r["y"])) for r in rows if r["state"] == state]
        if len(seg) == 1:
            seg = seg * 2
        sx, sy = zip(*seg)
        ax.plot(sx, sy, "-", color=STATE_COLORS.get(state, "black"),
                lw=2.2, label=f"actual: {state}")
    ax.plot(xs[0], ys[0], "s", color="tab:green", ms=10, label="start")
    ax.plot(*config.ZONE_B_CENTER, "*", color="tab:cyan", ms=15,
            label="Zone B (goal)")
    for defname, x, y, c in config.PACKAGES:
        ax.plot(x, y, "o", ms=6, color=c)

    ax.set_xlim(grid.x_min - 0.3, grid.x_max + 0.3)
    ax.set_ylim(grid.y_min - 0.3, grid.y_max + 0.3)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper left", fontsize=8)
    dur = float(rows[-1]["t"]) - float(rows[0]["t"])
    ax.set_title(f"Planned vs actual — full mission run ({dur:.0f} s, "
                 f"{len(rows)} samples)\n"
                 f"cross-track error: mean {sum(cte)/len(cte)*100:.1f} cm, "
                 f"max {max(cte)*100:.1f} cm")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"saved {out_path}")
    return info


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "logs", "run.csv")
    if not os.path.exists(csv_path):
        print(f"no run log at {csv_path} — run the mission in Webots first")
        sys.exit(1)
    plot_run(csv_path)


if __name__ == "__main__":
    main()
