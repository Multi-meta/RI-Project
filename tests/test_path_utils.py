"""Path utils tests: LOS simplification stays collision-free and shortens."""

import numpy as np
import pytest

from path_utils import (cells_to_world, line_of_sight, path_length,
                        resample_waypoints, simplify_path)
from mapping import OccupancyGrid


def test_simplify_shortens_and_is_safe():
    grid = OccupancyGrid(-2, 2, -2, 2, 0.1)
    grid.add_static_rect(0.0, 0.0, 0.4, 0.4)
    occ = grid.occupied()
    # an L-shaped detour around the obstacle
    path = [grid.world_to_cell(-1.0, 1.0),
            grid.world_to_cell(-0.35, 0.5),
            grid.world_to_cell(-0.3, 0.0),
            grid.world_to_cell(-0.3, -0.5),
            grid.world_to_cell(0.5, -1.0)]
    # make sure raw cells are free; nudge if a raw cell is occupied
    path = [c if not occ[c] else nearest_free_cell(occ, c) for c in path]
    simple = simplify_path(path, occ)
    assert len(simple) <= len(path)
    # simplified segments must never cross an occupied cell
    for (r0, c0), (r1, c1) in zip(simple[:-1], simple[1:]):
        assert line_of_sight(occ, (r0, c0), (r1, c1))


def nearest_free_cell(occ, cell):
    r, c = cell
    for dr in range(-5, 6):
        for dc in range(-5, 6):
            rr, cc = r + dr, c + dc
            if 0 <= rr < occ.shape[0] and 0 <= cc < occ.shape[1] \
                    and not occ[rr, cc]:
                return rr, cc
    raise AssertionError("no free cell")


def test_resample_keeps_endpoints():
    wps = [(0.0, 0.0), (1.0, 0.0)]
    out = resample_waypoints(wps, 0.3)
    assert out[0] == (0.0, 0.0)
    assert out[-1] == (1.0, 0.0)
    assert len(out) >= 4


def test_path_length():
    assert path_length([(0, 0), (3, 4)]) == pytest.approx(5.0)
    assert path_length([(1, 1)]) == 0.0


def test_cells_to_world_centers():
    grid = OccupancyGrid(0, 1, 0, 1, 0.1)
    (x, y) = cells_to_world([(5, 5)], grid)[0]
    assert x == pytest.approx(0.55) and y == pytest.approx(0.55)
