"""warehouse_grid.py — build the REAL warehouse occupancy grid from config.

Used by tests (Phase 1.4b assertions), tools (plot_astar), and the main
controller, so the grid is built exactly one way, in one place.
"""

from mapping import OccupancyGrid

import config


def build_warehouse_grid():
    """Known static map: arena bounds + walls + shelves + crates from config."""
    g = OccupancyGrid(config.ARENA_X_MIN, config.ARENA_X_MAX,
                      config.ARENA_Y_MIN, config.ARENA_Y_MAX, config.GRID_RES)
    for cx, cy, sx, sy in config.STATIC_OBSTACLES:
        g.add_static_rect(cx, cy, sx, sy)
    return g


def inflated_occupancy():
    """Boolean planning grid (True = blocked) for A*."""
    return build_warehouse_grid().inflate(config.INFLATION_RADIUS)
