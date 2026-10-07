"""a_star.py — 8-connected A* on a boolean occupancy grid.

Pure Python (heapq + numpy). Grid convention: boolean array, True = blocked.
Cells are (row, col).

Rules (project_summary.md §10.4):
    * 8-connected moves, cost 1 orthogonal / sqrt(2) diagonal
    * NO corner cutting: a diagonal move requires BOTH adjacent orthogonal
      cells to be free
    * octile heuristic: dx+dy + (sqrt(2)-2)*min(dx,dy)
    * returns (path, info) or (None, info); blocked start/goal are snapped to
      the nearest free cell (documented behavior)
    * info = {"cost", "expansions", "runtime_ms", "snapped", "found"}

Also provides `dijkstra`, used by tests to validate A* optimality on random
grids and available as the final-eval comparison algorithm.
"""

import heapq
import math
import time

import numpy as np

ORTHO = 1.0
DIAG = math.sqrt(2.0)
# (dr, dc, cost)
MOVES = [(-1, 0, ORTHO), (1, 0, ORTHO), (0, -1, ORTHO), (0, 1, ORTHO),
         (-1, -1, DIAG), (-1, 1, DIAG), (1, -1, DIAG), (1, 1, DIAG)]


def octile(dr, dc):
    adx, ady = abs(dr), abs(dc)
    return (adx + ady) + (DIAG - 2.0) * min(adx, ady)


def nearest_free(grid, row, col, max_radius=20):
    """BFS spiral outward to the nearest free cell. None if none within radius."""
    if not grid[row, col]:
        return row, col
    rows, cols = grid.shape
    for r in range(1, max_radius + 1):
        for dr in range(-r, r + 1):
            for dc in range(-r, r + 1):
                if max(abs(dr), abs(dc)) != r:      # only the ring boundary
                    continue
                rr, cc = row + dr, col + dc
                if 0 <= rr < rows and 0 <= cc < cols and not grid[rr, cc]:
                    return rr, cc
    return None


def a_star(grid, start, goal):
    """A* search. grid: bool ndarray (True = blocked); start/goal: (row, col).

    Returns (path_cells, info). path_cells is a list of (row, col) from start
    to goal inclusive, or None if unreachable.
    """
    t0 = time.perf_counter()
    info = {"cost": math.inf, "expansions": 0, "runtime_ms": 0.0,
            "snapped": False, "found": False}
    rows, cols = grid.shape

    start = tuple(start)
    goal = tuple(goal)
    for cell, name in ((start, "start"), (goal, "goal")):
        r, c = cell
        if not (0 <= r < rows and 0 <= c < cols) or grid[r, c]:
            nf = nearest_free(grid, r, c) if (0 <= r < rows and 0 <= c < cols) else None
            if nf is None:
                info["runtime_ms"] = (time.perf_counter() - t0) * 1000.0
                return None, info
            if name == "start":
                start = nf
            else:
                goal = nf
            info["snapped"] = True

    if start == goal:
        info.update(cost=0.0, found=True,
                    runtime_ms=(time.perf_counter() - t0) * 1000.0)
        return [start], info

    g = {start: 0.0}
    parent = {}
    # priority = g + eps*octile : octile is admissible & consistent for our
    # move costs; the tiny eps biases tie-breaking toward fewer expansions.
    eps = 1e-6
    open_heap = [(eps * octile(goal[0] - start[0], goal[1] - start[1]), 0.0, start)]
    closed = set()
    expansions = 0

    while open_heap:
        f, gcur, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)
        expansions += 1
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            path.reverse()
            info.update(cost=gcur, found=True, expansions=expansions,
                        runtime_ms=(time.perf_counter() - t0) * 1000.0)
            return path, info
        r, c = cur
        for dr, dc, cost in MOVES:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols) or grid[nr, nc]:
                continue
            if dr != 0 and dc != 0:
                # no corner cutting: both adjacent orthogonal cells must be free
                if grid[r + dr, c] or grid[r, c + dc]:
                    continue
            ng = gcur + cost
            nxt = (nr, nc)
            if ng < g.get(nxt, math.inf):
                g[nxt] = ng
                parent[nxt] = cur
                heapq.heappush(open_heap, (ng + eps * octile(goal[0] - nr, goal[1] - nc), ng, nxt))

    info["expansions"] = expansions
    info["runtime_ms"] = (time.perf_counter() - t0) * 1000.0
    return None, info


def dijkstra(grid, start, goal):
    """Uniform-cost search (h=0) on the same move set. Test reference and
    final-eval comparison baseline. Same return convention as a_star."""
    t0 = time.perf_counter()
    info = {"cost": math.inf, "expansions": 0, "runtime_ms": 0.0,
            "snapped": False, "found": False}
    rows, cols = grid.shape
    start, goal = tuple(start), tuple(goal)
    for cell, name in ((start, "start"), (goal, "goal")):
        r, c = cell
        if 0 <= r < rows and 0 <= c < cols and grid[r, c]:
            nf = nearest_free(grid, r, c)
            if nf is None:
                return None, info
            if name == "start":
                start = nf
            else:
                goal = nf
            info["snapped"] = True

    dist = {start: 0.0}
    parent = {}
    heap = [(0.0, start)]
    expansions = 0
    while heap:
        d, cur = heapq.heappop(heap)
        if d > dist.get(cur, math.inf):
            continue
        expansions += 1
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            path.reverse()
            info.update(cost=d, found=True, expansions=expansions,
                        runtime_ms=(time.perf_counter() - t0) * 1000.0)
            return path, info
        r, c = cur
        for dr, dc, cost in MOVES:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols) or grid[nr, nc]:
                continue
            if dr != 0 and dc != 0 and (grid[r + dr, c] or grid[r, c + dc]):
                continue
            nd = d + cost
            nxt = (nr, nc)
            if nd < dist.get(nxt, math.inf):
                dist[nxt] = nd
                parent[nxt] = cur
                heapq.heappush(heap, (nd, nxt))
    info["expansions"] = expansions
    info["runtime_ms"] = (time.perf_counter() - t0) * 1000.0
    return None, info
