"""Mapping tests: transforms, inflation, dynamic obstacles, blocked paths."""

import math

import numpy as np
import pytest

from mapping import (OccupancyGrid, bresenham, front_min_range,
                     obstacle_in_sector, path_is_blocked)
import config


@pytest.fixture
def grid():
    g = OccupancyGrid(-2.0, 2.0, -2.0, 2.0, 0.1)   # 40x40 cells
    g.add_static_rect(0.0, 0.0, 0.4, 0.4)          # 4x4 cells in the middle
    return g


def test_round_trip(grid):
    for x, y in [(-1.95, -1.95), (0.35, -0.71), (1.99, 0.03)]:
        r, c = grid.world_to_cell(x, y)
        xc, yc = grid.cell_to_world(r, c)
        assert math.hypot(xc - x, yc - y) < grid.res / 2 + 1e-9


def test_rect_marked(grid):
    r, c = grid.world_to_cell(0.0, 0.0)
    assert grid.static[r, c]
    r, c = grid.world_to_cell(0.3, 0.3)            # inside 0.4 box
    assert grid.static[r, c]
    r, c = grid.world_to_cell(0.3, 0.35)           # outside (corner)
    assert not grid.static[r, c]


def test_is_free_out_of_bounds(grid):
    assert not grid.is_free((100, 100))
    assert not grid.is_free((-1, 0))


def test_inflation_circular(grid):
    inf = grid.inflate(0.25)                       # 3-cell circular kernel
    r, c = grid.world_to_cell(0.0, 0.0)
    assert inf[r, c]
    # the obstacle spans cells r-2..r+2 / c-2..c+2. A cell 3 cells beyond the
    # CORNER (Euclidean 3*sqrt(2) cells from the corner cell) must NOT be
    # inflated, though a square kernel of radius 3 would have caught it.
    assert not inf[r + 5, c + 5]
    # a cell beyond the corner within 3 cells must be inflated
    assert inf[r + 3, c + 3]


def test_dynamic_obstacle(grid):
    grid.add_obstacle_world(1.0, 1.0, 0.15)
    r, c = grid.world_to_cell(1.0, 1.0)
    assert grid.dynamic[r, c]
    grid.clear_dynamic()
    assert not grid.dynamic.any()
    # static untouched
    r, c = grid.world_to_cell(0.0, 0.0)
    assert grid.static[r, c]


def test_path_is_blocked(grid):
    # path straight through the middle obstacle -> blocked
    path = [grid.world_to_cell(-1.0, 0.0), grid.world_to_cell(1.0, 0.0)]
    # densify through Bresenham inside path_is_blocked
    assert path_is_blocked(path, grid)
    # path far from obstacle -> free
    path = [grid.world_to_cell(-1.0, 1.5), grid.world_to_cell(1.0, 1.5)]
    assert not path_is_blocked(path, grid)
    # add dynamic obstacle on the clear path -> blocked (the final-eval hook)
    grid.add_obstacle_world(0.0, 1.5, 0.1)
    assert path_is_blocked(path, grid)


def test_lidar_to_world():
    grid = OccupancyGrid(-5, 5, -5, 5, 0.1)
    pose = (0.0, 0.0, 0.0)
    # hit straight ahead at 1 m and one 90 deg left at 2 m
    pts = grid.lidar_to_world(pose, [1.0, 2.0], [0.0, math.pi / 2])
    assert pts[0] == pytest.approx((1.0, 0.0))
    assert pts[1] == pytest.approx((0.0, 2.0), abs=1e-9)
    # rotated pose 90 deg CCW: forward hit now maps to +y
    pts = grid.lidar_to_world((0.0, 0.0, math.pi / 2), [1.0], [0.0])
    assert pts[0] == pytest.approx((0.0, 1.0), abs=1e-9)
    # invalid ranges dropped
    pts = grid.lidar_to_world(pose, [float("inf"), 0.5], [0.0, 0.0])
    assert len(pts) == 1


def test_sector_queries():
    angles = [math.radians(a) for a in range(-180, 180, 2)]
    scan = [4.0] * len(angles)
    # obstacle at 0 deg (index 90), 0.5 m; one at 40 deg (index 110), 1.2 m
    scan[90] = 0.5
    scan[110] = 1.2
    assert front_min_range(scan, angles, math.radians(30)) == pytest.approx(0.5)
    assert obstacle_in_sector(scan, angles, math.radians(35),
                              math.radians(45)) == pytest.approx(1.2)
    # sector with no obstacle: min of the clear 4.0 m rays
    assert obstacle_in_sector(scan, angles, math.radians(60),
                              math.radians(90)) == pytest.approx(4.0)


def test_bresenham():
    cells = bresenham(0, 0, 3, 3)
    assert (0, 0) in cells and (3, 3) in cells
    assert len(cells) == 4
    cells = bresenham(0, 0, 0, 5)
    assert cells == [(0, i) for i in range(6)]
