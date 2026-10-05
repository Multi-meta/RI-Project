"""plot_world_layout.py — clean top-down schematic of the warehouse layout
(designed world, from config) -> docs/evidence/world_layout.png.

Run:  python tools/plot_world_layout.py
"""

import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle  # noqa: E402

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(ROOT, "controllers", "intellibot_controller"))

import config  # noqa: E402

OUT = os.path.join(ROOT, "docs", "evidence", "world_layout.png")

PKG_RGB = {"red": "#d62828", "green": "#2a9d34", "blue": "#1d4ed8"}


def main():
    fig, ax = plt.subplots(figsize=(10, 7.5))
    ax.set_facecolor("#eef0f2")
    # arena + walls
    ax.add_patch(Rectangle((config.ARENA_X_MIN, config.ARENA_Y_MIN), 8, 6,
                           fill=False, edgecolor="#5b6672", lw=6))
    for x, y, sx, sy, h, d in config.SHELVES:
        ax.add_patch(FancyBboxPatch((x - sx / 2, y - sy / 2), sx, sy,
                                    boxstyle="round,pad=0.02", fc="#4b5563",
                                    ec="#374151"))
        ax.annotate(d, (x, y), ha="center", va="center", color="white",
                    fontsize=8)
    for x, y, sx, sy, h, d in config.CRATES:
        ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy,
                               fc="#a4713d", ec="#7c5227"))
        ax.annotate(d, (x, y), ha="center", va="center", color="white",
                    fontsize=7)
    # floor patches
    px, py, psx, psy = config.PICKUP_AREA
    ax.add_patch(Rectangle((px - psx / 2, py - psy / 2), psx, psy,
                           fc="#e5e7eb", ec="#9ca3af", ls="--"))
    ax.annotate("PICKUP", (px, py + 0.55), ha="center", color="#6b7280",
                fontsize=8)
    for zone, fc, label in ((config.ZONE_A, "#fde047", "ZONE A"),
                            (config.ZONE_B, "#67e8f9", "ZONE B")):
        zx, zy, zsx, zsy = zone
        ax.add_patch(Rectangle((zx - zsx / 2, zy - zsy / 2), zsx, zsy,
                               fc=fc, ec="#a1a1aa", alpha=0.85))
        ax.annotate(label, (zx, zy), ha="center", va="center",
                    color="#3f3f46", fontsize=9, fontweight="bold")
    for defname, x, y, color in config.PACKAGES:
        s = config.PKG_SIZE
        ax.add_patch(Rectangle((x - s / 2, y - s / 2), s, s,
                               fc=PKG_RGB[color], ec="black", lw=0.8, zorder=5))
        ax.annotate(defname, (x + 0.13, y), fontsize=7, va="center",
                    color="#111827")
    # robot start
    rx, ry, rth = config.ROBOT_START
    ax.add_patch(Circle((rx, ry), config.ROBOT_RADIUS, fc="#0ea5e9",
                        ec="#0369a1", zorder=6))
    ax.annotate("", xy=(rx + 0.3, ry), xytext=(rx, ry),
                arrowprops=dict(arrowstyle="-|>", color="#0369a1", lw=2),
                zorder=7)
    ax.annotate("INTELLIBOT", (rx, ry - 0.33), ha="center", fontsize=8,
                color="#0369a1", fontweight="bold")
    ax.set_xlim(-4.4, 4.4)
    ax.set_ylim(-3.4, 3.4)
    ax.set_aspect("equal")
    ax.set_title("IntelliBot warehouse — designed layout (from config.py)")
    ax.axis(False)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.tight_layout()
    fig.savefig(OUT, dpi=150)
    print(f"saved {OUT}")


if __name__ == "__main__":
    main()
