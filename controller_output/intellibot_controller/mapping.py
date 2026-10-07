"""mapping.py — occupancy grid, world<->grid transforms, inflation, LiDAR.

Pure Python / NumPy (no Webots).

Grid frame: cell (row, col), row indexes y (north), col indexes x (east).
    world -> grid : col = floor((x - x_min) / res), row = floor((y - y_min) / res)
    grid  -> world: center of the cell.

Conventions:
    * ``static``  : known map, built from config.STATIC_OBSTACLES (walls,
      shelves, crates). axis-aligned rectangles.
    * ``dynamic`` : LiDAR-detected obstacles (final eval). Held separately so
      the known map is never polluted.
    * occupied(grid) = static | dynamic
    * ``inflate(radius)`` uses a CIRCULAR kernel (distance-based), not square.

Coordinate transforms (robot->world) for LiDAR hits / camera detections:
    x_w = x_r + d*cos(theta + phi);  y_w = y_r + d*sin(theta + phi)
with phi the bearing in the robot frame (left = +, ENU CCW).
"""

import math

import numpy as np


class OccupancyGrid:
    def __init__(self, x_min, x_max, y_min, y_max, res):
        self.x_min, self.x_max = float(x_min), float(x_max)
        self.y_min, self.y_max = float(y_min), float(y_max)
        self.res = float(res)
        self.n_cols = int(math.ceil((self.x_max - self.x_min) / self.res))
        self.n_rows = int(math.ceil((self.y_max - self.y_min) / self.res))
        # row-major: shape (n_rows, n_cols); True = occupied
        self.static = np.zeros((self.n_rows, self.n_cols), dtype=bool)
        self.dynamic = np.zeros((self.n_rows, self.n_cols), dtype=bool)

    # ------------------------------------------------------------- transforms
    def world_to_cell(self, x, y):
        """World (m) -> (row, col) ints. May be out of bounds; check via in_bounds."""
        col = int(math.floor((x - self.x_min) / self.res))
        row = int(math.floor((y - self.y_min) / self.res))
        return row, col

    def cell_to_world(self, row, col):
        """(row, col) -> world coordinates of the cell CENTER."""
        x = self.x_min + (col + 0.5) * self.res
        y = self.y_min + (row + 0.5) * self.res
        return x, y

    def in_bounds(self, row, col):
        return 0 <= row < self.n_rows and 0 <= col < self.n_cols

    # ------------------------------------------------------------- occupancy
    def add_static_rect(self, cx, cy, sx, sy):
        """Mark an axis-aligned rectangle (center + full sizes) as static."""
        r0, c0 = self.world_to_cell(cx - sx / 2.0, cy - sy / 2.0)
        r1, c1 = self.world_to_cell(cx + sx / 2.0, cy + sy / 2.0)
        r0, r1 = max(r0, 0), min(r1, self.n_rows - 1)
        c0, c1 = max(c0, 0), min(c1, self.n_cols - 1)
        if r0 <= r1 and c0 <= c1:
            self.static[r0:r1 + 1, c0:c1 + 1] = True

    def add_obstacle_world(self, x, y, radius):
        """Mark a circular dynamic obstacle (final-eval hook)."""
        rr = int(math.ceil(radius / self.res))
        row, col = self.world_to_cell(x, y)
        for dr in range(-rr, rr + 1):
            for dc in range(-rr, rr + 1):
                if dr * dr + dc * dc <= (radius / self.res) ** 2:
                    r, c = row + dr, col + dc
                    if self.in_bounds(r, c):
                        self.dynamic[r, c] = True

    def clear_dynamic(self):
        self.dynamic[:] = False

    def occupied(self):
        """Combined (static | dynamic) occupancy array."""
        return self.static | self.dynamic

    def is_free(self, cell):
        """cell = (row, col). Out of bounds counts as NOT free."""
        r, c = cell
        if not self.in_bounds(r, c):
            return False
        return not (self.static[r, c] or self.dynamic[r, c])

    # ------------------------------------------------------------- inflation
    def inflate(self, radius):
        """Return a new boolean grid with obstacles grown by `radius` meters
        using a circular kernel."""
        rad_cells = int(math.ceil(radius / self.res))
        occ = self.occupied()
        if rad_cells <= 0:
            return occ.copy()
        out = occ.copy()
        # precompute circular kernel offsets
        for dr in range(-rad_cells, rad_cells + 1):
            for dc in range(-rad_cells, rad_cells + 1):
                if dr * dr + dc * dc > rad_cells * rad_cells:
                    continue
                if dr == 0 and dc == 0:
                    continue
                shifted = np.zeros_like(occ)
                rs = slice(max(dr, 0), self.n_rows + min(dr, 0))
                rd = slice(max(-dr, 0), self.n_rows + min(-dr, 0))
                cs = slice(max(dc, 0), self.n_cols + min(dc, 0))
                cd = slice(max(-dc, 0), self.n_cols + min(-dc, 0))
                shifted[rd, cd] = occ[rs, cs]
                out |= shifted
        return out

    # ------------------------------------------------------------- lidar
    def lidar_to_world(self, pose, ranges, angles):
        """LiDAR scan -> Nx2 array of world-frame hit points.

        pose   : (x, y, theta)
        ranges : array of ranges (m); inf/NaN/out-of-range are dropped
        angles : array of ray bearings in the ROBOT frame (rad, left=+)
        """
        x, y, theta = pose
        ranges = np.asarray(ranges, dtype=float)
        angles = np.asarray(angles, dtype=float)
        valid = np.isfinite(ranges) & (ranges >= 0.0)
        d = ranges[valid]
        phi = angles[valid]
        xs = x + d * np.cos(theta + phi)
        ys = y + d * np.sin(theta + phi)
        return np.column_stack((xs, ys))


def path_is_blocked(path_cells, grid):
    """True if any cell of the path is occupied on the CURRENT combined grid.

    Also checks the straight line between consecutive path cells with a
    Bresenham walk, so a re-planned path through a gap that later closed is
    detected even if the endpoints are free. Used by the final-eval
    re-planning loop; kept pure and tested now.
    """
    occ = grid.occupied()
    cells = list(path_cells)
    for r, c in cells:
        if not grid.in_bounds(r, c) or occ[r, c]:
            return True
    for (r0, c0), (r1, c1) in zip(cells[:-1], cells[1:]):
        for r, c in bresenham(r0, c0, r1, c1):
            if not grid.in_bounds(r, c) or occ[r, c]:
                return True
    return False


def bresenham(r0, c0, r1, c1):
    """All grid cells on the line from (r0,c0) to (r1,c1), inclusive."""
    cells = []
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = (r1 > r0) - (r1 < r0)
    sc = (c1 > c0) - (c1 < c0)
    err = dc - dr
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr
    return cells


def obstacle_in_sector(scan, angles, a_min, a_max):
    """Minimum valid range within bearing sector [a_min, a_max] (robot frame).

    Returns math.inf if the sector contains no valid ray.
    """
    scan = np.asarray(scan, dtype=float)
    angles = np.asarray(angles, dtype=float)
    a_min = wrap_angle(a_min)
    a_max = wrap_angle(a_max)
    if a_min <= a_max:
        mask = (angles >= a_min) & (angles <= a_max)
    else:  # sector wraps around +-pi
        mask = (angles >= a_min) | (angles <= a_max)
    vals = scan[mask & np.isfinite(scan)]
    return float(vals.min()) if vals.size else math.inf


def front_min_range(scan, angles, cone):
    """Minimum range in the forward cone +-`cone` radians."""
    return obstacle_in_sector(scan, angles, -cone, cone)


def wrap_angle(a):
    """Wrap angle to [-pi, pi)."""
    return math.atan2(math.sin(a), math.cos(a))
