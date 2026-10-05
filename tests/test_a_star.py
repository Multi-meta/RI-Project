"""A* tests: trivial, no-path, corner-cutting, blocked endpoints, Dijkstra
optimality on random grids."""

import numpy as np
import pytest

from a_star import a_star, dijkstra, nearest_free, octile
import math


def free_grid(n=20, m=20):
    return np.zeros((n, m), dtype=bool)


def test_trivial_path():
    g = free_grid()
    g[10, 5:15] = True                      # horizontal wall, detour needed
    path, info = a_star(g, (10, 0), (10, 19))
    assert path is not None
    assert info["found"]
    # optimality: cost must match Dijkstra on the same grid
    _, dinfo = dijkstra(g, (10, 0), (10, 19))
    assert info["cost"] == pytest.approx(dinfo["cost"], abs=1e-9)
    # and the path must not cross the wall
    for r, c in path:
        assert not g[r, c]


def test_start_equals_goal():
    g = free_grid()
    path, info = a_star(g, (5, 5), (5, 5))
    assert path == [(5, 5)]
    assert info["cost"] == 0.0


def test_no_path_returns_none():
    g = free_grid()
    g[:, 10] = True                          # full vertical wall
    path, info = a_star(g, (5, 5), (5, 15))
    assert path is None
    assert not info["found"]


def test_blocked_start_and_goal_snapped():
    g = free_grid()
    g[5, 5] = True
    g[10, 10] = True
    path, info = a_star(g, (5, 5), (10, 10))
    assert path is not None
    assert info["snapped"]
    assert path[0] != (5, 5) and path[-1] != (10, 10)
    assert nearest_free(g, 5, 5) == (4, 4)


def test_no_corner_cutting():
    g = free_grid()
    g[5, 5] = True
    g[4, 4] = False
    # moving diagonally from (4,5) to (5,4) would cut the corner of (5,5):
    # both orthogonal neighbors (5,4's left) ... construct explicitly:
    # blocked cell (5,5); diagonal step (4,4)->(5,5) needs (5,4) and (4,5)
    g[4, 5] = True                          # one orthogonal neighbor blocked
    path, info = a_star(g, (3, 3), (5, 4))
    assert path is not None
    # check no diagonal move in the path cuts a corner
    for (r0, c0), (r1, c1) in zip(path[:-1], path[1:]):
        if r0 != r1 and c0 != c1:
            assert not g[r1, c0] and not g[r0, c1]


def test_octile_heuristic():
    assert octile(3, 0) == pytest.approx(3.0)
    assert octile(3, 3) == pytest.approx(3 * math.sqrt(2))
    # max(dx,dy) + (sqrt2-1)*min(dx,dy) == dx+dy+(sqrt2-2)*min(dx,dy)
    assert octile(3, 1) == pytest.approx(3 + (math.sqrt(2) - 1) * 1)


def test_astar_equals_dijkstra_random():
    """Optimality: on 60 random grids A* cost must equal Dijkstra cost."""
    rng = np.random.default_rng(42)
    checked = 0
    for trial in range(60):
        g = rng.random((25, 25)) < 0.25      # 25% blocked
        start = (int(rng.integers(0, 25)), int(rng.integers(0, 25)))
        goal = (int(rng.integers(0, 25)), int(rng.integers(0, 25)))
        g[start] = False
        g[goal] = False
        pa, ia = a_star(g, start, goal)
        pd, id_ = dijkstra(g, start, goal)
        assert (pa is None) == (pd is None)
        if pa is not None:
            assert ia["cost"] == pytest.approx(id_["cost"], abs=1e-9)
            # verify path validity: every step legal and every cell free
            for (r0, c0), (r1, c1) in zip(pa[:-1], pa[1:]):
                assert max(abs(r1 - r0), abs(c1 - c0)) == 1
                assert not g[r1, c1]
                if r0 != r1 and c0 != c1:
                    assert not g[r0, c1] and not g[r1, c0]
            checked += 1
    assert checked >= 40                     # enough reachable cases ran
