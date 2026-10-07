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

- 2026-10-07 — Phase 6.3 — tools/plot_run.py + analyze_logs hook: planned vs
  actual overlay generated from the real logs/run.csv ->
  docs/evidence/planned_vs_actual.png (A* reference vs actual trajectory
  colored by FSM state, cross-track mean 0.13 cm / max 6.3 cm, 181 s run).
- 2026-10-07 — Phase 6.8 — campaign mode wired: RUN_INDEX=<i> teleports the
  supervisor robot to config.CAMPAIGN_STARTS[i] and logs to logs/run_<i>.csv;
  analyze_logs now emits docs/evidence/runs_summary.md (success rate, time,
  goal error per run) once campaign logs exist.
- 2026-10-07 — Phase 6.10/6.11 — display_dash.py: in-sim Display dashboard
  (map + route + robot + state, ~2 Hz) and LED state indicator
  (config.STATE_LED); devices added to both generated worlds.
- 2026-10-07 — Phase 7.2 — tools/check_evidence.py: evidence inventory with
  P0/P1 status -> docs/evidence/inventory.md. Remaining P0 gaps:
  lidar_scan.png, run_full.mp4 (both need Webots).
- 2026-10-07 — Phase 4.7/4.8 (partial) — tools/build_frame_gt.py derives
  ground truth for the captured spin frames from geometry (FOV + LOS check
  on the known map); eval_detector now reports 100% detection (6/6 package
  appearances at ~3.1 m), 0 false positives over 13 real frames. Small
  sample — more captures still needed for final rates.
- 2026-10-07 — Phase 4.9 (partial) — tools/eval_position.py: pinhole-fallback
  position error on the spin frames -> docs/evidence/position_estimate.md
  (mean 0.59 m at ~3.1 m, quantization-limited; LiDAR-fused number to be
  recorded live).
- 2026-10-07 — Phase 8 — deck updated with measured numbers (kinematics
  <=1%, odometry drift 2.2 cm/4.7 deg, full mission DONE in 181 s, detector
  100%/0 FP), real challenges (wheel contact, two-light fix, anti-windup),
  roadmap with RUN_INDEX campaign; demo_script.md gained the recording steps
  and per-slide presenter/rehearsal table. pytest: 63 passed.

- 2026-10-07 — Phase 0.4 — official R2025a sample worlds copied into worlds/
  audited for NODE structure (not just controllers): Lidar (tiltAngle/
  noise/type "rotating"), Camera+Recognition child, bare GPS/InertialUnit
  mounting, Display width/height, LED visual children, Pen leadSize,
  Connector autoLock (final-eval pickup), TouchSensor — verbatim snippets +
  reuse notes appended to docs/samples_audit.md ("World-file audit").
- 2026-10-07 — Phase 0.5 — tools/world_header.txt (minimal template, only
  base nodes; header byte-pattern proven by the running warehouse world) +
  worlds_output/trivial_test.wbt instantiated from it; worlds_output/ now
  always receives fresh copies from generate_world.py (previous copy was
  stale, from before the wheel/lighting fixes).
- 2026-10-07 — Phase 0.1 — env proof built into both controllers
  (print_env_versions / startup banner): every run prints python, numpy,
  cv2, matplotlib versions to the Webots console.
- 2026-10-07 — Phase 4.2/4.4 automation — drive_test `lidar` mode logs raw
  360-ray scans during one slow rotation (logs/lidar_raw.csv) and prints
  OBSTACLE AHEAD when the front cone closes (4.3); new
  tools/calibrate_lidar.py ray-marches the known map to determine
  LIDAR_ANGLE_SIGN/_OFFSET offline, writes the range-accuracy table
  (docs/evidence/lidar_validation.md) + logs/lidar_scan.csv and renders
  lidar_scan.png. Targets the 0.6 m package-estimate error (suspected
  mirrored ray order).

- 2026-10-07 (later) — CRITICAL FIX — the robot "not moving" in Webots: the
  whole controllers/intellibot_controller + drive_test folders had been
  deleted from the working tree, so Webots fell back to the <generic>
  controller ("The controller directory has not been found"). Restored from
  git and re-applied the uncommitted Phase 6–8 edits (config dashboard/LED/
  campaign sections, display_dash.py, mission wiring, drive_test lidar mode,
  env-version prints).
- 2026-10-07 (later) — Environment — 4 new small varied-shape obstacles in
  config + generator: 2 cylinders (barrels), 1 sphere (ball), 1 small crate,
  all in the central band so both outer corridors stay clear (test_warehouse
  still green). Forces A* to weave on every mission leg.
- 2026-10-07 (later) — BUG (found by the run): the ball was painted saturated
  blue and FIND_PACKAGE locked onto IT instead of package P3. Fixed: ball
  desaturated to grey. Lesson: environment colours are part of the
  detector's interface.
- 2026-10-07 (later) — Phase 4.2/4.4 MEASURED — DRIVE_TEST_MODE=lidar logged
  1321 raw scans; tools/calibrate_lidar.py vs the known map gives
  LIDAR_ANGLE_SIGN=-1, LIDAR_ANGLE_OFFSET=0.02 (index runs CLOCKWISE;
  median |err| 8 cm). This was the root cause of the 0.6 m package-estimate
  error. Config updated; docs/evidence/lidar_validation.md + lidar_scan.png
  produced.
- 2026-10-07 (later) — Mission re-run with fixes + new obstacles: P3 detected
  at (-2.99, -0.57) vs true (-3.10, -0.50) = 0.13 m error (was 0.6 m); full
  FIND->NAVIGATE->PICK->PLAN_DELIVERY->DELIVER->DONE in 69.2 s, 8.2 m,
  cross-track max 6.2 cm, final goal error 0.12 m. run.csv now ends at DONE
  (controller breaks its loop after DONE; simulationQuit alone kept batch
  runs alive to the 300 s timeout and buffered prints were lost — fixed with
  an explicit break + final CSV row).
- 2026-10-07 (later) — Snapshots re-taken (world_topdown/world_angle) with
  the new obstacles; metrics.md/planned_vs_actual regenerated; A* figure and
  world layout re-rendered for the new map (A* == Dijkstra, alt route OK).
- 2026-10-07 (later) — Phase 8 companion guide: docs/phase6_7_8_9_explained.md
  + Phase6_7_8_9_Explained.pdf (6 pages, LibreOffice via tools/md_to_docx.py)
  mirroring Phase2_3_Explained.pdf, including the problems-hit section and
  the run cheat sheet.
