"""
test_warehouse_v2.py — Verification of Warehouse v2 (Phase 13.1).

Verifies:
1. 12x8m arena dimensions and cell count.
2. Aisles >= 1.6 m wide for 2-robot clearance.
3. Path exists from receiving area to all shelves and dispatch zones.
4. Topologically distinct alternate routes exist.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import pytest
import numpy as np
import config
from warehouse_grid import build_warehouse_grid
from path_utils import plan_to


def test_v2_grid_dimensions():
    grid = build_warehouse_grid(v2=True)
    assert grid.x_min == -6.0 and grid.x_max == 6.0
    assert grid.y_min == -4.0 and grid.y_max == 4.0
    assert grid.n_cols == 120
    assert grid.n_rows == 80


def test_v2_aisles_width():
    """Aisle clearance test: central aisle and cross aisles must exceed 1.6 m."""
    grid = build_warehouse_grid(v2=True)
    occ = grid.inflate(config.V2_INFLATION_RADIUS)
    
    # Check central corridor: at x=0.0 from y=-1.5 to 1.5 must be free
    for y in np.linspace(-1.5, 1.5, 10):
        r, c = grid.world_to_cell(0.0, y)
        assert not occ[r, c], f"Central corridor blocked at y={y:.2f}"

    # Cross aisle at x=0.5 between north and south shelves
    for y in np.linspace(-3.0, 3.0, 20):
        r, c = grid.world_to_cell(0.5, y)
        assert not occ[r, c], f"Cross aisle blocked at y={y:.2f}"


def test_v2_routes_receiving_to_shelves_and_zones():
    """Paths must exist from receiving pile (-4.5, 0.0) to all shelf units and dispatch zones."""
    grid = build_warehouse_grid(v2=True)
    start = config.V2_RECEIVING_POS
    
    destinations = [
        (-1.5, 1.5),  # Front of SH1
        (2.5, 1.5),   # Front of SH2
        (-1.5, -1.5), # Front of SH3
        (2.5, -1.5),  # Front of SH4
        config.V2_DISPATCH_ZONE_A,
        config.V2_DISPATCH_ZONE_B,
    ]
    
    for dst in destinations:
        wps, info = plan_to(grid, start, dst, resample_spacing=0.3, inflation_radius=config.V2_INFLATION_RADIUS)
        assert wps is not None, f"No path found from receiving to {dst}!"
        assert info.get("found", False)
        assert info.get("length_m", 0) > 1.0


def test_v2_alternate_routes():
    """Blocking the central corridor must still allow reaching dispatch zones via outer aisles."""
    grid = build_warehouse_grid(v2=True)
    start = config.V2_RECEIVING_POS
    goal = config.V2_DISPATCH_ZONE_A
    
    wps1, info1 = plan_to(grid, start, goal, resample_spacing=0.3, inflation_radius=config.V2_INFLATION_RADIUS)
    assert wps1 is not None
    
    # Block central corridor at (0.0, 0.0)
    grid.add_obstacle_world(0.0, 0.0, 0.5)
    wps2, info2 = plan_to(grid, start, goal, resample_spacing=0.3, inflation_radius=config.V2_INFLATION_RADIUS)
    assert wps2 is not None, "Alternate route around central block failed!"
    assert info2.get("found", False)
