"""plot_astar.py — offline A* figure + optimality comparison (no Webots).

Builds the warehouse grid from config, plans pickup area -> Zone B, and
saves docs/evidence/astar_pickup_to_zoneB.png showing free / occupied /
inflated cells, start, goal, raw path, simplified waypoints, packages and
zones. Prints expansions/runtime/length vs Dijkstra.

Run:  python tools/plot_astar.py
"""

import math
import os
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(ROOT, "controllers", "intellibot_controller"))

import config  # noqa: E402
from a_star import a_star, dijkstra  # noqa: E402
from path_utils import simplify_path, cells_to_world, path_length  # noqa: E402
from warehouse_grid import build_warehouse_grid  # noqa: E402

OUT = os.path.join(ROOT, "docs", "evidence", "astar_pickup_to_zoneB.png")


def main():
    grid = build_warehouse_grid()
    occ = grid.inflate(config.INFLATION_RADIUS)

    start_xy = config.ROBOT_START[:2]
    goal_xy = config.ZONE_OF[config.TASK_ZONE]
    start = grid.world_to_cell(*start_xy)
    goal = grid.world_to_cell(*goal_xy)

    t0 = time.perf_counter()
    path, info = a_star(occ, start, goal)
    t_astar = (time.perf_counter() - t0) * 1000
    if path is None:
        print("A*: NO PATH FOUND — check layout!")
        sys.exit(1)
    simple = simplify_path(path, occ)
    wps = cells_to_world(simple, grid)
    length = path_length(wps)

    t0 = time.perf_counter()
    dpath, dinfo = dijkstra(occ, start, goal)
    t_dij = (time.perf_counter() - t0) * 1000

    print(f"A*:       cost={info['cost']:.3f} cells "
          f"({info['cost'] * config.GRID_RES:.2f} m; {length:.2f} m simplified), "
          f"{info['expansions']} expansions, {t_astar:.1f} ms")
    print(f"Dijkstra: cost={dinfo['cost']:.3f} cells, "
          f"{dinfo['expansions']} expansions, {t_dij:.1f} ms")
    same = abs(info["cost"] - dinfo["cost"]) < 1e-6
    print(f"Optimality: A* cost == Dijkstra cost -> {same}")
    print(f"Waypoints: raw {len(path)} -> simplified {len(simple)} "
          f"-> resampled (0.3 m) {max(2, int(length / 0.3))}")

    # ---------------- plot ----------------
    fig, ax = plt.subplots(figsize=(10, 7.5))
    occ_img = np.ma.masked_where(~occ, occ)
    ax.imshow(occ_img, cmap="Reds", origin="lower",
              extent=[grid.x_min, grid.x_max, grid.y_min, grid.y_max],
              alpha=0.9, vmin=0, vmax=2)
    static_img = np.ma.masked_where(~grid.static, grid.static)
    ax.imshow(static_img, cmap="Greys", origin="lower",
              extent=[grid.x_min, grid.x_max, grid.y_min, grid.y_max],
              alpha=0.9, vmin=0, vmax=2)

    pr = np.array([grid.cell_to_world(r, c) for r, c in path])
    ax.plot(pr[:, 0], pr[:, 1], ".", color="tab:orange", ms=3,
            label="A* raw path")
    sw = np.array(wps)
    ax.plot(sw[:, 0], sw[:, 1], "-o", color="tab:blue", ms=4, lw=1.6,
            label="simplified waypoints")
    ax.plot(*start_xy, "s", color="tab:green", ms=11,
            label=f"start ({start_xy[0]}, {start_xy[1]})")
    ax.plot(*goal_xy, "*", color="tab:cyan", ms=15,
            label=f"Zone B ({goal_xy[0]}, {goal_xy[1]})")
    for defname, x, y, c in config.PACKAGES:
        ax.plot(x, y, "o", ms=7, color=c)
        ax.annotate(defname, (x, y), textcoords="offset points", xytext=(6, 6),
                    fontsize=8)
    for zone, col, label in ((config.ZONE_A, "gold", "ZONE_A"),
                             (config.ZONE_B, "deepskyblue", "ZONE_B")):
        zx, zy, zsx, zsy = zone
        ax.add_patch(plt.Rectangle((zx - zsx / 2, zy - zsy / 2), zsx, zsy,
                                   fill=False, edgecolor=col, lw=2))
        ax.annotate(label, (zx, zy), color=col, fontsize=8)

    ax.set_xlim(grid.x_min - 0.3, grid.x_max + 0.3)
    ax.set_ylim(grid.y_min - 0.3, grid.y_max + 0.3)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper left", fontsize=8)
    ax.set_title(f"A* on warehouse grid  |  res {config.GRID_RES} m, inflation "
                 f"{config.INFLATION_RADIUS} m  |  cost "
                 f"{info['cost'] * config.GRID_RES:.2f} m, "
                 f"{info['expansions']} expansions")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.tight_layout()
    fig.savefig(OUT, dpi=150)
    print(f"saved {OUT}")

    # two distinct routes check (final-eval hook): block center aisle
    grid2 = build_warehouse_grid()
    grid2.add_obstacle_world(0.0, 0.0, 1.4)   # seal the middle
    occ2 = grid2.inflate(config.INFLATION_RADIUS)
    p2, i2 = a_star(occ2, start, goal)
    print(f"Center aisle blocked: path still exists -> {p2 is not None} "
          f"(cost {i2.get('cost', float('nan')) * config.GRID_RES:.2f} m)")


if __name__ == "__main__":
    main()
