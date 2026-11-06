"""display_dash.py — in-sim Display dashboard + LED state indicator.

Implements checklist 6.10 (P2) and 6.11 (P2). The Dashboard draws the
occupancy map, the current waypoint route, the robot and the FSM state onto
a Webots `Display` device; `set_state_led` mirrors the FSM state onto an LED
using config.STATE_LED colors.

Only this module touches the Display/LED devices; drawing is called at a
low rate (~2 Hz) from the main loop to keep control timing clean.
"""

import math

import config

# display palette (0xRRGGBB for Webots Display.setColor)
COL_BG = 0xF4F5F7
COL_OCC = 0x3A4656
COL_INFL = 0xF3C9B4
COL_ROUTE = 0xEA580C
COL_ROBOT = 0x1668B4
COL_TEXT = 0x1E293B
COL_PKG = 0x1D4ED8


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
    def draw(self, pose, state, sim_time, waypoints=None, pkg_xy=None):
        self._draw_map()
        self._draw_route(waypoints)
        if pkg_xy:
            self._draw_pkg(pkg_xy)
        self._draw_robot(pose)
        self._draw_text(state, sim_time)

    def _draw_map(self):
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
        self.d.setFont("Arial", 12, True)
        self.d.drawText(f"STATE: {state}", 4, 14)
        self.d.setFont("Arial", 10, False)
        self.d.drawText(f"t = {sim_time:6.1f} s", 4, 28)


def set_state_led(led, state):
    """Mirror the FSM state onto the LED (checklist 6.11)."""
    if led is not None:
        led.set(config.STATE_LED.get(state, 0))
