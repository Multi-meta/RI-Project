"""path_utils.py — path post-processing: simplification, resampling, metrics.

Pure Python / NumPy (no Webots).

Pipeline from A*: raw grid cells -> Bresenham line-of-sight simplification
(collapse collinear runs into straight, collision-free segments) -> optional
resampling at fixed spacing (light "trajectory generation") -> world waypoints
(cell centers).
"""

import math

import numpy as np

from mapping import bresenham


def line_of_sight(grid, a, b):
    """True if the straight grid line a->b crosses no occupied cell."""
    for r, c in bresenham(a[0], a[1], b[0], b[1]):
        if grid[r, c]:
            return False
    return True


def simplify_path(path_cells, grid):
    """Keep only waypoints that are NECESSARY: walk from the last kept cell
    as far as line-of-sight allows. Output never crosses an occupied cell
    (given the same grid)."""
    if len(path_cells) <= 2:
        return list(path_cells)
    out = [path_cells[0]]
    i = 0
    while i < len(path_cells) - 1:
        j = len(path_cells) - 1
        while j > i + 1 and not line_of_sight(grid, path_cells[i], path_cells[j]):
            j -= 1
        out.append(path_cells[j])
        i = j
    return out


def resample_waypoints(waypoints, spacing):
    """Resample a polyline of world (x, y) points at ~`spacing` meters.
    Keeps endpoints exactly. spacing <= 0 disables."""
    if spacing is None or spacing <= 0 or len(waypoints) < 2:
        return list(waypoints)
    pts = [np.asarray(p, dtype=float) for p in waypoints]
    out = [pts[0]]
    carry = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        seg = b - a
        seg_len = float(np.hypot(*seg))
        if seg_len < 1e-9:
            continue
        direction = seg / seg_len
        pos = carry
        while pos < seg_len:
            out.append(a + direction * pos)
            pos += spacing
        carry = pos - seg_len
    if np.hypot(*(out[-1] - pts[-1])) > 1e-9:
        out.append(pts[-1])
    return [(float(p[0]), float(p[1])) for p in out]


def cells_to_world(path_cells, grid):
    """Grid cells -> world waypoints (cell centers)."""
    return [grid.cell_to_world(r, c) for r, c in path_cells]


def path_length(waypoints):
    """Total Euclidean length of a world waypoint polyline (m)."""
    total = 0.0
    for (x0, y0), (x1, y1) in zip(waypoints[:-1], waypoints[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def plan_to(grid, start_xy, goal_xy, resample_spacing=0.3,
            inflation_radius=None):
    """End-to-end planner glue: snap -> A* -> simplify -> world waypoints.

    Returns (waypoints_world, info) or (None, info). `grid` is the raw
    OccupancyGrid; when `inflation_radius` is given the planning grid is the
    combined occupancy inflated by that radius. info carries A* stats and the
    final path length.
    """
    from a_star import a_star  # local import keeps module import graph flat

    if inflation_radius:
        occupancy = grid.inflate(inflation_radius)
    else:
        occupancy = grid.occupied()
    start_cell = grid.world_to_cell(*start_xy)
    goal_cell = grid.world_to_cell(*goal_xy)
    path, info = a_star(occupancy, start_cell, goal_cell)
    if path is None:
        return None, info
    simple = simplify_path(path, occupancy)
    wps = cells_to_world(simple, grid)
    wps = resample_waypoints(wps, resample_spacing)
    info["n_waypoints_raw"] = len(path)
    info["n_waypoints"] = len(wps)
    info["length_m"] = path_length(wps)
    return wps, info
