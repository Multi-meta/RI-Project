"""dynamic_map.py — continuous LiDAR obstacle layer with observed-free clearing.

Writes mapping.OccupancyGrid.dynamic (the layer the A* planner already reads
through grid.inflate()); the known static map is never modified.

Each update (throttled to DYN_MAP_RATE_HZ):

* every valid LiDAR ray is transformed to the world with the ESTIMATED pose
  (x_w = x + r cos(theta + phi), y_w = y + r sin(theta + phi));
* the end cell of a ray that hit something gains evidence (+DYN_HIT_GAIN)
  unless it is explained by the static map (a wall / shelf / crate cell or
  its 1-cell neighbourhood, which absorbs small pose errors) — that is how
  static and dynamic obstacles are told apart;
* every cell the ray passed THROUGH loses evidence (-DYN_MISS_LOSS): cells
  observed free are cleared, so an obstacle that moved away disappears (no
  ghost obstacles) — but a cell that is simply not observed (behind the
  robot) keeps its evidence, which is what stops the planner from choosing
  the route that was just found blocked;
* cells unobserved for DYN_TTL seconds expire;
* a cell is occupied when evidence >= DYN_OCCUPIED_LEVEL (two hits: a single
  noisy return never blocks a route);
* the robot's own footprint and caller-given keep-out circles (the target
  parcel during the approach) are never marked.

Updates are skipped while localisation is unhealthy (a wrong pose would paint
obstacles in the wrong place).
"""

import math

import numpy as np

import config


class DynamicObstacleMap:
    def __init__(self, grid):
        self.grid = grid
        self.evidence = np.zeros((grid.n_rows, grid.n_cols), dtype=np.float32)
        self.last_seen = np.full((grid.n_rows, grid.n_cols), -np.inf)
        self.last_cleared = np.full((grid.n_rows, grid.n_cols), -np.inf)
        # static cells grown by one cell: hits there are the known map
        st = grid.static
        grown = st.copy()
        grown[1:, :] |= st[:-1, :]
        grown[:-1, :] |= st[1:, :]
        grown[:, 1:] |= st[:, :-1]
        grown[:, :-1] |= st[:, 1:]
        self.static_near = grown
        self._t_last = -np.inf
        self._written = np.zeros_like(grid.dynamic)
        self._pins = np.zeros_like(grid.dynamic)
        self.updates = 0
        self.marked_total = 0

    # ----------------------------------------------------------------- update
    def update(self, pose, scan, angles, t, keep_out=(), trusted=True, force=False):
        """Fuse one scan. keep_out: [(x, y, radius)] never marked occupied."""
        if scan is None or angles is None or not trusted:
            return False
        if not force and t - self._t_last < 1.0 / config.DYN_MAP_RATE_HZ:
            return False
        self._t_last = t
        g = self.grid
        was_occ = self.evidence >= config.DYN_OCCUPIED_LEVEL
        x, y, th = pose
        r = np.asarray(scan, dtype=float)
        a = np.asarray(angles, dtype=float)
        rmax = min(config.DYN_MAP_MAX_RANGE, config.LIDAR_MAX_RANGE)
        valid = ~np.isnan(r)
        hit = valid & np.isfinite(r) & (r < rmax) & (r > config.LIDAR_SELF_FILTER_RANGE)
        free_len = np.where(hit, r - g.res, np.where(valid, rmax, 0.0))
        free_len = np.clip(free_len, 0.0, rmax)

        # ---- free space along every ray (vectorised sampling at res/2)
        step = g.res * 0.5
        n_s = int(math.ceil(rmax / step))
        ds = (np.arange(n_s) + 0.5) * step
        ang = th + a
        px = x + np.outer(np.cos(ang), ds)
        py = y + np.outer(np.sin(ang), ds)
        mask = ds[None, :] < free_len[:, None]
        cols = np.floor((px[mask] - g.x_min) / g.res).astype(int)
        rows = np.floor((py[mask] - g.y_min) / g.res).astype(int)
        ib = (rows >= 0) & (rows < g.n_rows) & (cols >= 0) & (cols < g.n_cols)
        if np.any(ib):
            flat = np.unique(rows[ib] * g.n_cols + cols[ib])
            fr, fc = np.divmod(flat, g.n_cols)
            self.evidence[fr, fc] = np.maximum(0.0, self.evidence[fr, fc] - config.DYN_MISS_LOSS)
            self.last_seen[fr, fc] = t

        # ---- hits
        hx = x + r[hit] * np.cos(ang[hit])
        hy = y + r[hit] * np.sin(ang[hit])
        hc = np.floor((hx - g.x_min) / g.res).astype(int)
        hr = np.floor((hy - g.y_min) / g.res).astype(int)
        ib = (hr >= 0) & (hr < g.n_rows) & (hc >= 0) & (hc < g.n_cols)
        hx, hy, hr, hc = hx[ib], hy[ib], hr[ib], hc[ib]
        keep = ~self.static_near[hr, hc]
        for (kx, ky, kr) in keep_out:
            keep &= np.hypot(hx - kx, hy - ky) > kr
        if np.any(keep):
            flat = np.unique(hr[keep] * g.n_cols + hc[keep])
            rr, cc = np.divmod(flat, g.n_cols)
            self.evidence[rr, cc] = np.minimum(config.DYN_MAX_EVIDENCE,
                                               self.evidence[rr, cc] + config.DYN_HIT_GAIN)
            self.last_seen[rr, cc] = t

        # ---- expiry of long-unobserved cells, own footprint, keep-outs
        stale = (t - self.last_seen) > config.DYN_TTL
        self.evidence[stale & (self.evidence > 0)] = 0.0
        self._clear_disc(x, y, config.ROBOT_RADIUS)
        for (kx, ky, kr) in keep_out:
            self._clear_disc(kx, ky, kr)
        # anything another module wrote into grid.dynamic since our last
        # update (e.g. non-target parcels seen by the camera) is kept ("pinned")
        self._pins |= g.dynamic & ~self._written
        occ = self.evidence >= config.DYN_OCCUPIED_LEVEL
        # an occupied cell seen free again = something MOVED out of it
        self.last_cleared[was_occ & ~occ] = t
        newly = int(np.count_nonzero(occ & ~g.dynamic))
        self.marked_total += newly
        g.dynamic[:] = occ | self._pins
        self._written = g.dynamic.copy()
        self.updates += 1
        return True

    def _clear_disc(self, x, y, radius):
        g = self.grid
        row, col = g.world_to_cell(x, y)
        rc = int(math.ceil(radius / g.res))
        r0, r1 = max(0, row - rc), min(g.n_rows, row + rc + 1)
        c0, c1 = max(0, col - rc), min(g.n_cols, col + rc + 1)
        if r0 >= r1 or c0 >= c1:
            return
        rr, cc = np.mgrid[r0:r1, c0:c1]
        cx = g.x_min + (cc + 0.5) * g.res
        cy = g.y_min + (rr + 0.5) * g.res
        m = np.hypot(cx - x, cy - y) <= radius
        self.evidence[rr[m], cc[m]] = 0.0

    def clear(self):
        self.evidence[:] = 0.0
        self.last_seen[:] = -np.inf
        self.last_cleared[:] = -np.inf
        self._pins[:] = False
        self._written[:] = False
        self.grid.dynamic[:] = False

    def moving_near(self, x, y, radius, t, window=None, min_cells=2):
        """True when at least `min_cells` cells within `radius` of (x, y) were
        occupied and then observed free in the last `window` s: the obstacle
        there is MOVING (a crossing cart / person), not a static blockage."""
        window = config.DYN_MOVING_WINDOW if window is None else window
        g = self.grid
        row, col = g.world_to_cell(x, y)
        rc = int(math.ceil(radius / g.res))
        r0, r1 = max(0, row - rc), min(g.n_rows, row + rc + 1)
        c0, c1 = max(0, col - rc), min(g.n_cols, col + rc + 1)
        if r0 >= r1 or c0 >= c1:
            return False
        recent = (t - self.last_cleared[r0:r1, c0:c1]) <= window
        return int(np.count_nonzero(recent)) >= min_cells

    def evidence_occupied(self):
        """Cells occupied by LiDAR evidence only (not pinned marks)."""
        return self.evidence >= config.DYN_OCCUPIED_LEVEL

    # ---------------------------------------------------------------- queries
    def occupied_points(self):
        """World (x, y) of the occupied dynamic cells (dashboard)."""
        rows, cols = np.nonzero(self.grid.dynamic)
        return [self.grid.cell_to_world(r, c) for r, c in zip(rows, cols)]

    def count(self):
        return int(np.count_nonzero(self.grid.dynamic))
