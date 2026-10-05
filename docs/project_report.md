# Project Report — IntelliBot (mid-evaluation skeleton)

> Skeleton to be extended into the final report. Numbers marked ⏳ are
> pending simulator runs; fill them from `docs/evidence/*` — never invent.

## 1. Introduction
IntelliBot is a software-only Webots simulation of a differential-drive
warehouse robot performing "pick P3 (blue), deliver to Zone B (cyan)" with a
closed perceive→decide→plan→control loop. Scope: mid-evaluation world,
perception, planning, control, FSM integration; final evaluation adds real
pickup, dynamic obstacles with live re-planning, and dashboard.

## 2. Architecture
See `docs/architecture.md` (block diagram, module paragraphs, A/B/C
ownership). Key design rules: Webots-free core, config.py as single source
of truth, world generated from config.

## 3. Methods
- **Kinematics/odometry:** differential-drive model, curvature-preserving
  clamping, arc-model odometry vs GPS+IMU.
- **Perception:** HSV package detection (pure OpenCV), LiDAR sector queries,
  pixel→bearing→range→world position estimation.
- **Planning:** occupancy grid (0.1 m), circular inflation (0.25 m), A*
  (8-connected, octile, no corner cutting), Bresenham simplification,
  0.3 m resampling.
- **Control:** PID heading + speed shaping waypoint follower, turn-in-place,
  LiDAR safety stop.
- **Decision:** FSM with printed transitions, unit-tested with simulated
  unicycle.

## 4. Results
| Metric | Value | Source |
|---|---|---|
| Unit tests | 63 passed | pytest |
| A* == Dijkstra cost (60 random grids) | yes | test_a_star.py |
| A* start→Zone B | 3.06 m simplified, 1849 expansions, ~14 ms | plot_astar.py |
| Alternate route when center blocked | exists (verified) | test_warehouse.py |
| Kinematics error ⏳ | pending | docs/evidence/kinematics_validation.md |
| Odometry drift ⏳ | pending | docs/evidence/odometry_vs_gps.png |
| LiDAR accuracy ⏳ | pending | docs/evidence/lidar_*.md |
| Detector rates ⏳ | pending | docs/evidence/detector_eval.md |
| Cross-track error / success rate ⏳ | pending | docs/evidence/runs_summary.md |

## 5. Limitations
GPS+IMU localization (idealized); pickup placeholder; dynamic obstacles not
spawned live; detector validated on synthetic frames so far; Webots runs on
the build machine pending — all sim-dependent numbers above marked ⏳.

## 6. Future work (final evaluation)
Real pickup (connector/supervisor attach), dynamic obstacle + live re-plan
(hooks tested), PID tuning w/ step response, collision counter, in-sim
dashboard, Dijkstra/RRT comparison, detector robustness campaign.
