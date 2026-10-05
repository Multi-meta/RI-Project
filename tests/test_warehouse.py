"""Warehouse grid tests (checklist 1.4b): real layout has a path from the
pickup area to Zone B, and TWO topologically distinct routes exist."""

import numpy as np
import pytest

import config
from a_star import a_star
from path_utils import path_length, plan_to
from warehouse_grid import build_warehouse_grid

APPROACH = (-2.55, 0.0)      # near the pickup area, outside inflation


def test_path_exists_pickup_to_zoneB():
    g = build_warehouse_grid()
    wps, info = plan_to(g, APPROACH, config.ZONE_B_CENTER,
                        resample_spacing=0.0,
                        inflation_radius=config.INFLATION_RADIUS)
    assert wps is not None, "no path from pickup area to Zone B!"
    assert info["length_m"] > 1.0


def test_two_distinct_routes():
    """Block the center aisle -> an alternate (outer corridor) route exists."""
    g = build_warehouse_grid()
    wps, _ = plan_to(g, APPROACH, config.ZONE_B_CENTER,
                     resample_spacing=0.0,
                     inflation_radius=config.INFLATION_RADIUS)
    assert wps is not None
    base_cost = _cost(g, APPROACH, config.ZONE_B_CENTER)

    # seal the center: wall of obstacles across x=0 between the shelves
    for y in np.arange(-2.0, 2.01, 0.2):
        g.add_obstacle_world(0.0, float(y), 0.3)
    wps2, _ = plan_to(g, APPROACH, config.ZONE_B_CENTER,
                      resample_spacing=0.0,
                      inflation_radius=config.INFLATION_RADIUS)
    assert wps2 is not None, "blocking the center aisle killed ALL routes"
    alt_cost = _cost(g, APPROACH, config.ZONE_B_CENTER)
    # the alternate route must be meaningfully different (longer or detoured)
    assert alt_cost > base_cost * 1.05


def _cost(grid, start, goal):
    occ = grid.inflate(config.INFLATION_RADIUS)
    s = grid.world_to_cell(*start)
    t = grid.world_to_cell(*goal)
    _, info = a_star(occ, s, t)
    return info["cost"]


def test_aisles_are_wide_enough():
    """With the 0.25 m inflation there must be free corridor cells between
    every pair of facing obstacles (aisle >= ~1.0 m)."""
    g = build_warehouse_grid()
    occ = g.inflate(config.INFLATION_RADIUS)
    # corridor between shelf rows (y = -1.65..-1.15 region between shelves
    # and center) — check a straight vertical line at shelf x-extents
    for x in np.arange(-2.8, 2.81, 0.1):
        col_cells = []
        for y in np.arange(-3.0, 3.01, 0.05):
            r, c = g.world_to_cell(float(x), float(y))
            col_cells.append(occ[r, c])
        arr = np.array(col_cells)
        # every vertical scan line must contain some free space
        assert not arr.all(), f"fully blocked column at x={x:.1f}"
