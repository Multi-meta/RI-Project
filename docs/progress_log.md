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

- 2026-10-06 — Phase 2.1 — Webots R2025a found at D:\RI_CyberBotics\Webots and
  driven from the CLI. Generator fixed: `utf8` header, colliding floor,
  WorldInfo caster ContactProperties, unique solid names, DYN_OBSTACLE moved
  off the south wall. Writes warehouse.wbt + warehouse_drive_test.wbt
  (same world, robot controller = drive_test). Opens with 0 warnings.
- 2026-10-06 — Phase 2.2 — drive_test `snapshot` mode (supervisor sets the
  Viewpoint, `exportImage`) → docs/evidence/world_topdown.png, world_angle.png.
- 2026-10-06 — Phase 2.3 — drive_test `worldcheck` mode: "world matches config".
- 2026-10-06 — Phase 3.1 — casters given collision + physics; `stability`
  mode: 10 s at rest, 0.00 mm drift, 0.000 deg tilt.
- 2026-10-06 — Phase 3.2 — sequence run: forward = +x, spin-left = +yaw (ENU
  confirmed). drive_test rewritten (Supervisor, repo-root log paths, fixed
  `writerows` header bug, per-second pose print).
- 2026-10-06 — Phase 3.3 — first run showed ω −14.2 % (cylinder wheel contact
  at outer rim ⇒ L_eff = L + width; confirmed with half-width wheels −7.6 %).
  Wheel boundingObject → Sphere: v, ω now within −0.9 %. Measurement switched
  to per-step signed/wrapped differences (old chord method lost the sign on
  reverse and wrapped on long spins). Table: docs/evidence/kinematics_validation.md.
- 2026-10-06 — Phase 3.4 — odometry re-initialised to GPS pose per trajectory;
  drift 0.8 cm line / 2.2 cm + 4.7 deg square / 1.1 cm arc →
  docs/evidence/odometry_vs_gps.png, metrics.md (analyze_logs.py updated).
- 2026-10-06 — Phase 3.5 [P2] — Pen trail NOT done (needs a textured floor).
- 2026-10-06 — Found for Phase 6: intellibot_controller/controller.py shadows
  the Webots `controller` module → mission controller import will fail
  until it is renamed.
- 2026-10-06 — pytest: **63 passed** (Python 3.11).

- 2026-10-06 — Rename — intellibot_controller/controller.py ->
  waypoint_follower.py (it shadowed the Webots `controller` module so the
  mission controller could not start). Mission log now written to repo-root
  logs/run.csv.
- 2026-10-06 — World lighting — first mission run: "package never found".
  drive_test `capture` mode showed package faces at HSV value ~46 (one light,
  faces toward the robot unlit). Added a second opposed directional light;
  all 3 packages detected. Screenshots (2.2) retaken.
- 2026-10-06 — First full mission run — IDLE→FIND→NAVIGATE→PICK→DELIVER→DONE
  in 61.7 s sim time, 7.14 m, replans=0. Package estimate 0.6 m short
  (suspected LiDAR ray order, Phase 4.2). Not yet ticked in the checklist.

## Pending simulator runs (human operator)

Everything in README "Run" section: open world, drive_test (sequence/
kinematics/odometry modes), full mission run, LiDAR calibration (4.2),
detector frame capture (4.7). Numbers must be filled into docs/evidence/*
from real logs — do not invent.
