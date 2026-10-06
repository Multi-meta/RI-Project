# IntelliBot Architecture

## Block diagram

```
                    ┌────────────────────────── Webots simulator ──────────────────────────┐
                    │                                                                      │
   sensors ──►  perception.py ──► decision_maker.py (FSM) ──► planner (A*) ──► waypoint_follower.py
   (LiDAR,       (device         (state logic, goals)       (path_utils     (WaypointFollower,
   camera,       wrappers +      │                           on occupancy     PID heading)
   GPS/IMU,      BGRA→BGR,       │                           grid)               │
   encoders)     ray angles)     │                                                 v
                    │            └──────────── logged transitions ──► logger.py    kinematics.py
                    │                                                              (v,ω → wheel ω)
                    ▼                                                                    │
                 motors ◄────────────────────────────────────────────────────────────────┘
```

Data flow per control step (16 ms):
`sensors → perception (pose, scan, detections) → decision_maker (FSM state)
→ planner (A* → waypoints, only when a new goal is set) → controller
(waypoint + pose → v, ω) → kinematics (v, ω → wheel speeds) → motors → logger`.

## Modules and how to explain them (viva-ready)

> Ownership: **A — Simulation & Control**, **B — Perception**, **C — Planning
> & Decisions** (project_summary.md §14). Each owner must be able to explain
> their modules AND the interfaces to the other two.

### config.py — [shared, maintained by C]
**What:** every constant in one file: arena layout (`STATIC_OBSTACLES`),
robot dimensions, device names, grid resolution, control gains, HSV ranges,
FSM thresholds, task definition.
**How to explain:** "The world generator, the occupancy grid, the robot
description and the controller all read the same numbers from this file, so
the simulated world and the planner's map can never disagree. No magic
numbers anywhere else."

### kinematics.py — [A]
**What:** differential-drive forward/inverse kinematics with
curvature-preserving wheel-speed clamping, `wrap_to_pi`, and the
`Odometry` class (arc model, NaN-safe first reading).
**Interface:** `inverse_kinematics(v, ω) → (ωL, ωR)` is called every step by
the main controller; `Odometry.update(encoder angles)` produces the
odometry pose compared against GPS+IMU.
**Explain:** derive v = r(ωR+ωL)/2, ω = r(ωR−ωL)/L on the board; clamping
scales BOTH wheels by the same factor so the robot turns along the same
curved path, just slower.

### pid.py — [A]
**What:** PID with output limits and conditional-integration anti-windup
(integrator frozen while saturated).
**Interface:** `update(error, dt) → u`; used by WaypointFollower for heading.

### mapping.py — [C]
**What:** `OccupancyGrid` (static known layer + dynamic LiDAR layer),
world↔cell transforms (cell centers, floor division), circular-kernel
inflation, `add_obstacle_world`/`clear_dynamic`, `path_is_blocked`,
`lidar_to_world` (polar→Cartesian through the robot pose), sector queries.
**Explain:** "world_to_cell: col = floor((x−x_min)/res), row = floor((y−y_min)/res);
cell_to_world returns the cell CENTER. Inflation grows every obstacle by
robot radius + margin with a circular kernel — a square kernel would close
diagonal gaps that the robot can actually pass."

### a_star.py — [C]
**What:** 8-connected A* with octile heuristic, no corner cutting, tie-break
epsilon; returns path + info (cost, expansions, runtime) or None cleanly;
blocked start/goal snapped to nearest free cell. `dijkstra` included as the
optimality reference (test) and final-eval comparison.
**Explain:** "octile distance is the exact grid distance when diagonals cost
√2 — admissible and consistent, so A* is optimal; we PROVED it by matching
Dijkstra's cost on 60 random grids in pytest."

### path_utils.py — [C]
**What:** Bresenham line-of-sight simplification (collapses the raw cell path
to few collision-free waypoints), 0.3 m resampling ("trajectory generation"),
`cells_to_world`, `path_length`, and the end-to-end `plan_to` glue.
**Interface:** `plan_to(grid, start, goal, inflation) → (waypoints, info)` —
the only planning call the FSM makes.

### warehouse_grid.py — [C]
**What:** builds the REAL warehouse occupancy grid from `config.STATIC_OBSTACLES`
— one place, used by tests, tools, and the controller.

### detector.py — [B]
**What:** pure-OpenCV HSV detector: per-color masks (red = two hue ranges),
morphological open+close, largest contour with area ≥ 150 px and plausible
aspect ratio, `Detection` with a documented HEURISTIC score (solidity × size
factor — NOT a statistical confidence).
**Explain:** "HSV separates color from brightness, so it survives moderate
lighting changes; red wraps around hue 0, hence two ranges; zones are
yellow/cyan, chosen outside the red/green/blue hue windows."

### estimation.py — [B]
**What:** the spatial transformation chain pixel→world: image column →
bearing (pinhole model, left = +) → LiDAR range at that bearing (fallback:
`d = f_px·PKG_SIZE/bbox_width`) → world position via pose + polar transform.
**Explain:** "This is the robotics-vision + frames exercise: a 2D pixel gets
depth from an independent sensor (LiDAR) and is transformed into the world
frame with the robot's pose."

### perception.py — [B]
**What:** the only Webots-facing perception code. Enables devices, wraps
GPS+IMU pose, encoder odometry, LiDAR (`getRangeImage` → NumPy, invalid =
inf, measured ray-angle mapping via `LIDAR_ANGLE_SIGN`), camera
(BGRA→BGR ndarray), detections, and the supervisor world/config consistency
check.
**Interface:** `get_pose()`, `get_lidar()`, `get_detections()`,
`set_wheel_speeds()`.

### waypoint_follower.py — [A]
**What:** `WaypointFollower` — PID heading control, speed shaping
(cos-error × slowdown), turn-in-place threshold, waypoint/goal tolerances,
cross-track error metric, LiDAR safety stop (front ±30° < 0.30 m → v=0,
status BLOCKED — the final-eval re-planning hook).
**Explain:** "v and ω are decoupled: ω steers the heading error to zero with
PID; v is throttled by how aligned we are and how close the waypoint is."

### decision_maker.py — [C]
**What:** the FSM: IDLE→FIND_PACKAGE→NAVIGATE→PICK(placeholder)→
PLAN_DELIVERY→DELIVER→DONE (+ERROR). Every transition printed. Testable
without Webots — planner and estimator are injected callables.
**Explain:** "the FSM is pure logic; pytest drives it with a simulated
unicycle through the whole mission, including the package-never-found ERROR
path."

### intellibot_controller.py — [C]
**What:** entry point: wires perception → FSM → planner → kinematics →
motors → logger at 16 ms; ~10 Hz CSV logging; AUTO_QUIT ends simulation at
DONE/ERROR.

### logger.py — [shared]
**What:** the §13 CSV columns (pose, odometry, commands, state, waypoint
index, cross-track error, front LiDAR min, detection count).

## Tools (offline, no Webots needed)

| Tool | Output |
|---|---|
| `tools/generate_world.py` | `worlds/warehouse.wbt` (world = config, always) |
| `tools/plot_astar.py` | `docs/evidence/astar_pickup_to_zoneB.png` + A*/Dijkstra comparison + two-route check |
| `tools/plot_lidar.py` | `docs/evidence/lidar_scan.png` (needs `logs/lidar_scan.csv` from sim) |
| `tools/eval_detector.py` | `docs/evidence/detector_eval.md` + annotated example (needs saved frames) |
| `tools/analyze_logs.py` | `docs/evidence/metrics.md` + odometry overlay plot |
