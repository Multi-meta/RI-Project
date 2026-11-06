"""warehouse_grid.py — build the REAL warehouse occupancy grid from config.

Used by tests (Phase 1.4b assertions), tools (plot_astar), and the main
controller, so the grid is built exactly one way, in one place.
"""

from mapping import OccupancyGrid

import config


def build_warehouse_grid(v2=False):
    """Known static map: arena bounds + walls + shelves + crates from config."""
    if v2:
        g = OccupancyGrid(config.V2_ARENA_X_MIN, config.V2_ARENA_X_MAX,
                          config.V2_ARENA_Y_MIN, config.V2_ARENA_Y_MAX, config.GRID_RES)
        for cx, cy, sx, sy in config.V2_STATIC_OBSTACLES:
            g.add_static_rect(cx, cy, sx, sy)
        return g
    g = OccupancyGrid(config.ARENA_X_MIN, config.ARENA_X_MAX,
                      config.ARENA_Y_MIN, config.ARENA_Y_MAX, config.GRID_RES)
    for cx, cy, sx, sy in config.STATIC_OBSTACLES:
        g.add_static_rect(cx, cy, sx, sy)
    return g


def inflated_occupancy(v2=False):
    """Boolean planning grid (True = blocked) for A*."""
    rad = config.V2_INFLATION_RADIUS if v2 else config.INFLATION_RADIUS
    return build_warehouse_grid(v2=v2).inflate(rad)
