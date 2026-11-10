"""display_dash.py — in-sim Display dashboard + LED state indicator.

Implements checklist 6.10 (P2) and 6.11 (P2). The Dashboard draws the
occupancy map, the current waypoint route, the robot and the FSM state onto
a Webots `Display` device; `set_state_led` mirrors the FSM state onto an LED
using config.STATE_LED colors.

Only this module touches the Display/LED devices; drawing is called at a
low rate (~2 Hz) from the main loop to keep control timing clean.

MEMORY LEAK FIX (2026-11-10): Webots R2025a leaks memory on EVERY
Display.setFont() call. The dashboard called it 3x per refresh, and
webots-bin grew ~60 MB/s (measured: private bytes 0.6 -> 10 GB) until it
aborted with "unknown software exception 0x40000015" a few minutes into a GUI
run; the controller then died with MemoryError. Bisected in batch runs:
map/shapes/text drawn with a single setFont() stay flat at ~600 MB.
So the font is set ONCE (first draw) and never again. The static map is also
rasterised once and pasted as one image instead of ~2000 fillRectangle calls
per refresh.
"""

import math

import numpy as np

import config

try:                                   # Webots Display pixel format
    from controller import Display as _WbDisplay
    _BGRA = _WbDisplay.BGRA
except Exception:                      # unit tests / no Webots
    _BGRA = 6

# display palette (0xRRGGBB for Webots Display.setColor)
COL_BG = 0xF4F5F7
COL_OCC = 0x3A4656
COL_INFL = 0xF3C9B4
COL_ROUTE = 0xEA580C
COL_ROBOT = 0x1668B4
COL_TEXT = 0x1E293B
COL_PKG = 0x1D4ED8
COL_DYN = 0xDC2626        # LiDAR dynamic obstacles
COL_GT = 0x16A34A         # ground-truth marker (evaluation only)
COL_WARN = 0xB91C1C


class Dashboard:
    def __init__(self, display, grid):
        self.d = display
        self.w = display.getWidth()
        self.h = display.getHeight()
        self.grid = grid
        # precompute the static inflated occupancy ONCE (dynamic layer is a
        # final-eval concern; refresh() recomputes when wired)
        self.occ = grid.inflate(config.INFLATION_RADIUS)
        self.cols = grid.n_cols
        self.rows = grid.n_rows
        self.cell = min(self.w / self.cols, self.h / self.rows)
        self.ox = (self.w - self.cols * self.cell) / 2.0
        self.oy = (self.h - self.rows * self.cell) / 2.0
        self._map_img = None           # Display ImageRef of the static map (made once)
        self._map_failed = False
        self._font_set = False         # setFont() leaks in Webots: call it ONCE

    # ------------------------------------------------------------- helpers
    def _cell_rect(self, row, col):
        x = self.ox + col * self.cell
        y = self.oy + (self.rows - 1 - row) * self.cell   # display y grows down
        return x, y, self.cell, self.cell

    def _world_px(self, x, y):
        u = self.ox + (x - self.grid.x_min) / self.grid.res * self.cell
        v = self.oy + (self.grid.y_max - y) / self.grid.res * self.cell
        return u, v

    # --------------------------------------------------------------- draw
    def draw(self, pose, state, sim_time, waypoints=None, pkg_xy=None, extra=None):
        """extra (optional): dict with
            dyn_points   [(x, y)] dynamic obstacle cells
            gt_pose      (x, y, th) ground truth — evaluation overlay only
            lines        [str] status lines (localisation, sensors, replans,
                         speed cap, shelf occupancy) drawn under the state
            warn         bool — draw the status lines in red
        """
        extra = extra or {}
        if not self._font_set:
            self.d.setFont("Arial", 9, False)
            self._font_set = True
        self._draw_map()
        self._draw_dynamic(extra.get("dyn_points"))
        self._draw_route(waypoints)
        if pkg_xy:
            self._draw_pkg(pkg_xy)
        if extra.get("gt_pose") is not None:
            u, v = self._world_px(extra["gt_pose"][0], extra["gt_pose"][1])
            self.d.setColor(COL_GT)
            self.d.drawOval(int(u) - 5, int(v) - 5, 10, 10)
        self._draw_robot(pose)
        self._draw_text(state, sim_time)
        self._draw_lines(extra.get("lines") or [], extra.get("warn", False))

    def _draw_dynamic(self, pts):
        if not pts:
            return
        self.d.setColor(COL_DYN)
        for x, y in pts:
            r, c = self.grid.world_to_cell(x, y)
            px, py, cw, ch = self._cell_rect(r, c)
            self.d.fillRectangle(int(px), int(py), int(math.ceil(cw)), int(math.ceil(ch)))

    def _draw_lines(self, lines, warn):
        self.d.setColor(COL_WARN if warn else COL_TEXT)
        for i, line in enumerate(lines[:8]):
            self.d.drawText(str(line)[:48], 4, 42 + 12 * i)

    def _map_raster(self):
        """Static map (background + inflation + obstacles) as BGRA bytes."""
        img = np.empty((self.h, self.w, 4), dtype=np.uint8)

        def bgra(c):
            return ((c & 0xFF), (c >> 8) & 0xFF, (c >> 16) & 0xFF, 0xFF)

        img[:, :] = bgra(COL_BG)
        for layer, col in ((self.occ, COL_INFL), (self.grid.static, COL_OCC)):
            px = bgra(col)
            rows, cols = np.nonzero(layer)
            for r, c in zip(rows, cols):
                x, y, cw, ch = self._cell_rect(r, c)
                x0, y0 = int(x), int(y)
                img[max(y0, 0):max(y0 + int(math.ceil(ch)), 0),
                    max(x0, 0):max(x0 + int(math.ceil(cw)), 0)] = px
        return img.tobytes()

    def _draw_map(self):
        if self._map_img is None and not self._map_failed:
            try:
                self._map_img = self.d.imageNew(self._map_raster(), _BGRA, self.w, self.h)
            except Exception:
                self._map_failed = True        # fall back to primitives below
        if self._map_img is not None:
            self.d.imagePaste(self._map_img, 0, 0, False)
            return
        self.d.setColor(COL_BG)
        self.d.fillRectangle(0, 0, self.w, self.h)
        self.d.setColor(COL_INFL)
        for r in range(self.rows):
            for c in range(self.cols):
                if self.occ[r, c]:
                    x, y, cw, ch = self._cell_rect(r, c)
                    self.d.fillRectangle(int(x), int(y),
                                         int(math.ceil(cw)), int(math.ceil(ch)))
        self.d.setColor(COL_OCC)
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid.static[r, c]:
                    x, y, cw, ch = self._cell_rect(r, c)
                    self.d.fillRectangle(int(x), int(y),
                                         int(math.ceil(cw)), int(math.ceil(ch)))

    def _draw_route(self, waypoints):
        if not waypoints:
            return
        self.d.setColor(COL_ROUTE)
        pts = [self._world_px(x, y) for x, y in waypoints]
        for (u0, v0), (u1, v1) in zip(pts[:-1], pts[1:]):
            self.d.drawLine(int(u0), int(v0), int(u1), int(v1))
        for u, v in pts:
            self.d.fillOval(int(u) - 2, int(v) - 2, 4, 4)

    def _draw_pkg(self, pkg_xy):
        u, v = self._world_px(*pkg_xy)
        self.d.setColor(COL_PKG)
        self.d.fillOval(int(u) - 3, int(v) - 3, 6, 6)

    def _draw_robot(self, pose):
        u, v = self._world_px(pose[0], pose[1])
        self.d.setColor(COL_ROBOT)
        self.d.fillOval(int(u) - 4, int(v) - 4, 8, 8)
        # heading tick
        self.d.drawLine(int(u), int(v),
                        int(u + 10 * math.cos(pose[2])),
                        int(v - 10 * math.sin(pose[2])))

    def _draw_text(self, state, sim_time):
        self.d.setColor(COL_TEXT)
        self.d.drawText(f"STATE: {state}", 4, 14)          # font set once (see top)
        self.d.drawText(f"t = {sim_time:6.1f} s", 4, 28)


def set_state_led(led, state):
    """Mirror the FSM state onto the LED (checklist 6.11)."""
    if led is not None:
        led.set(config.STATE_LED.get(state, 0))
