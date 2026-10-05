# Progress Log

Format: date — checklist item — 1–3 lines.

- 2026-10-05 — Phase 0.1 — Environment recon: Webots R2025a registered but the
  application executable is NOT on disk (only asset cache); Python 3.12.3 with
  numpy/opencv/matplotlib/pytest all importable. Simulator runs delegated to
  the human operator with exact steps (README + docs/demo_script.md).
- 2026-10-05 — Phase 0.2 — Repo skeleton created (controllers/intellibot_controller,
  drive_test, tools, tests, docs/evidence, logs, worlds).
- 2026-10-05 — Phase 0.3 — docs/samples_audit.md written: all 26 sample folders
  audited with exact API calls to reuse (enable/getRangeImage/getImage/
  getRollPitchYaw/supervisor getFromDef/connector lock ...).
- 2026-10-05 — Phase 1.1 — config.py written: ALL constants (layout, robot,
  devices, grid, control, HSV, FSM, task, final-eval hooks).
- 2026-10-05 — Phase 1.2 — kinematics.py + 8 tests: fwd/inv round trip,
  curvature-preserving clamping, wrap_to_pi, odometry line/circle/NaN-safe.
- 2026-10-05 — Phase 1.3 — pid.py + 6 tests: P/I/D terms, output clamp,
  conditional-integration anti-windup (replaced a broken back-calculation
  scheme that made the heading controller stall — integral jumps of ±40 with
  ki=0.05).
- 2026-10-05 — Phase 1.4 — mapping.py + 8 tests: transforms round-trip,
  circular inflation, dynamic obstacles, path_is_blocked, lidar_to_world,
  sector queries, Bresenham.
- 2026-10-05 — Phase 1.5 — a_star.py + 7 tests: trivial/no-path/snap/
  corner-cutting/octile; optimality == Dijkstra on 60 random grids.
- 2026-10-05 — Phase 1.4b — warehouse_grid.py + test_warehouse.py: real layout
  has a pickup→Zone B path; blocking the center aisle still leaves an alternate
  (outer corridor) route; aisle-width scan passes.
- 2026-10-05 — Phase 1.6 — path_utils.py + 5 tests: LOS simplification is
  collision-free and shortens, resampling keeps endpoints, path_length.
- 2026-10-05 — Phase 1.7 — tools/plot_astar.py run: A* cost 3.25 m (grid) /
  3.06 m simplified from start to Zone B, 1849 expansions, == Dijkstra cost;
  figure saved to docs/evidence/astar_pickup_to_zoneB.png. Center-aisle-blocked
  check: alternate path exists.
- 2026-10-05 — Phase 2.1 — tools/generate_world.py written; worlds/warehouse.wbt
  generated from config (primitives only, no EXTERNPROTO; no shadows;
  basicTimeStep 16; DYN_OBSTACLE parked non-colliding). Opens-clean check
  pending (needs Webots — see README).
- 2026-10-05 — Phase 3.1 — Robot node generated inside warehouse.wbt: body,
  2 driven wheels (HingeJoint+RotationalMotor+PositionSensor), 2 zero-friction
  casters, Lidar (360 rays, 2π, 0.05–4 m), Camera (320×240, FOV 1.0), GPS,
  InertialUnit. Stability watch pending (needs Webots).
- 2026-10-05 — Phase 3.2 — controllers/drive_test/drive_test.py: scripted
  sequence + kinematics validation (GPS/IMU finite differences →
  logs/kinematics_test.csv) + odometry trajectories (line/square/arc →
  logs/odometry.csv). Sim runs pending.
- 2026-10-05 — Phase 4.1–4.3 — perception.py: LiDAR wrapper (validity mask,
  measured-angle hooks LIDAR_ANGLE_SIGN/OFFSET), sector queries, camera BGRA→
  BGR, GPS+IMU pose, odometry, world/config consistency check.
- 2026-10-05 — Phase 4.6 — detector.py + 6 tests on synthetic frames: correct
  color/bbox, red two-range handling, zone colors NOT confused with packages,
  small blobs rejected, annotate() draws.
- 2026-10-05 — Phase 4.9 — estimation.py + 6 tests: pixel→bearing→LiDAR
  range→world chain incl. rotated pose and pinhole fallback.
- 2026-10-05 — Phase 5.1 — controller.py WaypointFollower + 7 tests: reaches
  single/multi-waypoint paths in a simulated unicycle, rotates first when
  misaligned, LiDAR safety stop sets BLOCKED, slowdown + cross-track metrics.
- 2026-10-05 — Phase 6.4 — decision_maker.py FSM + 6 tests: happy path
  IDLE→…→DONE with mission report, package-never-found→ERROR, planner-failure
  →ERROR, blocked-hook behavior, all transitions printed.
- 2026-10-05 — Phase 6.9 — test_replan_hooks.py: dynamic obstacle on the route
  → path_is_blocked fires → re-plan finds a clear alternate route; clear_dynamic
  restores. NOT wired live (final-eval wow feature).
- 2026-10-05 — Phase 7.1 — tools/analyze_logs.py + tools/eval_detector.py +
  tools/plot_lidar.py written (ready for sim logs).
- 2026-10-05 — pytest: **63 passed**.

## Pending simulator runs (human operator)

Everything in README "Run" section: open world, drive_test (sequence/
kinematics/odometry modes), full mission run, LiDAR calibration (4.2),
detector frame capture (4.7). Numbers must be filled into docs/evidence/*
from real logs — do not invent.
