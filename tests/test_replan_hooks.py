"""Final-eval re-plan hooks (checklist 6.9): verified by test, NOT wired
live. Blocking a planned path must be detectable and an alternate route
must be plannable."""

import numpy as np
import pytest

import config
from a_star import a_star
from mapping import path_is_blocked
from path_utils import plan_to
from warehouse_grid import build_warehouse_grid


def test_replan_hook_end_to_end():
    grid = build_warehouse_grid()
    start = config.ROBOT_START[:2]
    goal = config.ZONE_B_CENTER

    wps, info = plan_to(grid, start, goal, resample_spacing=0.0,
                        inflation_radius=config.INFLATION_RADIUS)
    assert wps is not None
    # plan_to returns world waypoints; get the cell path for the blocked check
    occ = grid.inflate(config.INFLATION_RADIUS)
    cells = [grid.world_to_cell(x, y) for x, y in wps]
    assert not path_is_blocked(cells, grid)      # clear initially

    # dynamic obstacle lands ON the route (center of the arena corridor)
    mid = wps[len(wps) // 2]
    grid.add_obstacle_world(mid[0], mid[1], 0.35)
    assert path_is_blocked(cells, grid), "blocked route not detected!"

    # re-plan: A* must find an alternate route around the new obstacle
    wps2, info2 = plan_to(grid, start, goal, resample_spacing=0.0,
                          inflation_radius=config.INFLATION_RADIUS)
    assert wps2 is not None, "no alternate route after blocking"
    cells2 = [grid.world_to_cell(x, y) for x, y in wps2]
    assert not path_is_blocked(cells2, grid)     # new route is clear


def test_clear_dynamic_restores_route():
    grid = build_warehouse_grid()
    start, goal = config.ROBOT_START[:2], config.ZONE_B_CENTER
    wps, _ = plan_to(grid, start, goal, resample_spacing=0.0,
                     inflation_radius=config.INFLATION_RADIUS)
    cells = [grid.world_to_cell(x, y) for x, y in wps]
    mid = wps[len(wps) // 2]
    grid.add_obstacle_world(mid[0], mid[1], 0.3)
    assert path_is_blocked(cells, grid)
    grid.clear_dynamic()
    assert not path_is_blocked(cells, grid)
