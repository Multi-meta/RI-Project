# CHECKLIST.md — Build IntelliBot up to the Mid-Evaluation

> **How to use:** Read `project_summary.md` first (spec, conventions, rules). Then work through this file **top to bottom**. Tick each box `- [x]` only when its **Done when** check passes. After each task add 1–3 lines to `docs/progress_log.md`.
>
> **Priority tags:** `[P0]` must-have · `[P1]` should-have · `[P2]` nice-to-have.
> **Time is short.** Phases 0–6 are ordered so that a working demo exists as early as possible. See the **CUT LINE** after Phase 6: if time runs out, stop adding features there and go straight to Phase 7 (evidence) and Phase 8 (deck/docs).
>
> **If you can't run Webots yourself:** finish the code + tests, then give the human exact steps ("open `worlds/warehouse.wbt`, press ▶, paste the console output / save screenshot X to `docs/evidence/`") and wait for results. Never claim a simulation result you did not observe.

---

## Phase 0 — Setup & reconnaissance

- [ ] **0.1 [P0] Environment check.** Record in `docs/webots_findings.md`: Webots version, OS, Python version used by Webots (Preferences → Python command), and whether `numpy`, `cv2`, `matplotlib` import inside a controller.
  *Done when:* a throwaway controller prints `numpy`, `cv2`, `matplotlib` versions in the Webots console.
- [x] **0.2 [P0] Create repo skeleton** exactly as in `project_summary.md` §9 (empty modules with docstrings, `tests/`, `tools/`, `docs/`, `docs/evidence/`, `logs/`, `requirements.txt`, `README.md` stub). Do **not** modify existing sample folders in `controllers/`.
  *Done when:* `pytest` runs (0 tests, no errors) and the tree matches the spec.
- [x] **0.3 [P0] Samples audit.** Open the `.py` file in each of these sample folders and write `docs/samples_audit.md` (one short section each: what it shows, key API calls to copy, which world it uses): `lidar`, `camera`, `camera_recognition`, `gps`, `inertial_unit`, `position_sensor`/`encoders`, `motor`, `distance_sensor`, `sample_supervisor`, `gps_supervisor`, `connector`, `vacuum_gripper`, `display`, `pen`, `emitter_receiver`. (`hokuyo`, `sick`, `camera_segmentation`, `bumper` — skim only.)
  *Done when:* the audit file has an entry per folder with at least the exact calls we will reuse (e.g., `lidar.enable`, `getRangeImage`, `getHorizontalResolution`, `getFov`, `getMaxRange`).
- [ ] **0.4 [P1] Open 2–3 sample worlds** from the Webots installation (`projects/samples/devices/worlds/`, e.g. the lidar and camera_recognition worlds) to see how devices are mounted/configured. Note any node snippets worth copying (Lidar/Camera node fields, `Recognition` node) into the audit file.
  *Done when:* snippets are recorded.
- [ ] **0.5 [P0] Verify world-file requirements** for the installed version: `#VRML_SIM` header, `coordinateSystem "ENU"`, whether `EXTERNPROTO` lines are required. Save a minimal working header template to `tools/world_header.txt`.
  *Done when:* a trivial world (floor + one box) generated from the template opens without errors.

---

## Phase 1 — Pure-Python core (no Webots needed) [P0]

Build and test these first; they are independent of the simulator.

- [x] **1.1 `config.py`.** All constants from `project_summary.md` §6–§10: robot dims, motor limits, device names, grid params, thresholds, HSV ranges (initial guesses), `STATIC_OBSTACLES`, object centers, package/zone definitions, task (`P3` → `B`), tolerances.
  *Done when:* every module imports constants from here; no duplicated numbers.
- [x] **1.2 `kinematics.py` + tests.** `forward_kinematics(wl, wr)`, `inverse_kinematics(v, w)` with wheel-speed clamping that preserves curvature, `wrap_to_pi`, `Odometry` class (§10.1).
  *Done when:* tests pass for: straight line (wl=wr), spin in place (wl=−wr), round-trip `inverse→forward` returns (v, ω), clamping preserves ω/v ratio, odometry integrates a known circle/line correctly.
- [x] **1.3 `pid.py` + tests.** PID with output limits and integral anti-windup (§10.2).
  *Done when:* tests cover proportional response, integral accumulation + clamp, derivative sign, reset.
- [x] **1.4 `mapping.py` + tests.** `OccupancyGrid`, `world_to_cell`/`cell_to_world`, circular-kernel inflation, `add_obstacle_world`, `clear_dynamic`, `path_is_blocked`, `lidar_to_world` (§10.3).
  *Done when:* tests cover: round-trip conversion error < res/2; inflation grows obstacle by the right number of cells; out-of-bounds handling; `path_is_blocked` true/false cases; `lidar_to_world` for a hit straight ahead and 90° left at a rotated pose.
- [x] **1.5 `a_star.py` + tests.** 8-connected, no corner cutting, octile heuristic, returns path + info or `None` (§10.4).
  *Done when:* tests: trivial path, start==goal, no-path returns `None`, blocked start/goal handled, corner-cutting forbidden, and **cost equals Dijkstra on ≥50 random grids**.
- [x] **1.4b [P0] Build the real warehouse grid** from `STATIC_OBSTACLES`, inflate by 0.25 m, and confirm: pickup area → Zone B has a path; **two distinct routes exist** (block the center aisle in a test and a path still exists).
  *Done when:* a test asserts both properties on the actual warehouse layout.
- [x] **1.6 `path_utils.py` + tests.** Bresenham line-of-sight simplification, optional resampling/smoothing, grid→world waypoints, `path_length` (§10.5).
  *Done when:* simplified path never crosses an obstacle (test) and is shorter than/equal in waypoint count to the raw path.
- [x] **1.7 `tools/plot_astar.py`.** Matplotlib figure: free/occupied/inflated cells, start, goal, raw path, simplified waypoints, zones/packages annotated. Save `docs/evidence/astar_pickup_to_zoneB.png`. Also print expansions/runtime/length vs Dijkstra.
  *Done when:* the PNG exists and looks clear enough for a slide.

---

## Phase 2 — Warehouse world [P0]

- [ ] **2.1 `tools/generate_world.py`.** Reads `config.py` and writes `worlds/warehouse.wbt` using the header from 0.5: arena + walls, 4 shelves, 2 crates, pickup patch, packages P1–P3 (pure red/green/blue, each with `DEF`, `boundingObject`, `physics`), Zone A (yellow) / Zone B (cyan) patches (no collision), flat lighting with **no cast shadows**, `basicTimeStep 16`. Add a (non-colliding or far-parked) `DEF DYN_OBSTACLE` only if trivial — otherwise leave for final.
  *Done when:* the world opens in Webots with zero warnings in the console and matches the layout table in `project_summary.md` §7.
- [ ] **2.2 [P0] Visual check + evidence.** Take a top-down screenshot and an angled screenshot → `docs/evidence/world_topdown.png`, `world_angle.png`.
  *Done when:* files exist; all objects are where `config.py` says.
- [ ] **2.3 [P1] World/config consistency check** (runs from the controller via Supervisor at startup, behind a config flag): for each `DEF` object compare `getPosition()` with `config.py`; warn on mismatch > 1 cm.
  *Done when:* check prints "world matches config" on a clean run.

---

## Phase 3 — Robot, locomotion, kinematics validation [P0]

- [ ] **3.1 Robot node** (inside the generator, or as a separate `tools/robot_def` snippet merged by it): `DEF INTELLIBOT Robot { supervisor TRUE, controller "intellibot_controller" }` with body, 2 wheels (HingeJoint + RotationalMotor + PositionSensor + wheel Solid with physics), 2 zero-friction passive casters, `Lidar`, `Camera`, `GPS`, `InertialUnit`, using device names from `config.py` (spec §8).
  *Done when:* robot sits stably on the floor at start with no jitter or drift when motors are 0 (watch for 10 s).
- [ ] **3.2 `controllers/drive_test/drive_test.py`** — minimal controller (separate folder) that sets motors in velocity mode and runs a scripted sequence: forward 2 s, spin left 2 s, arc, reverse, stop; prints GPS/IMU pose each second.
  *Done when:* robot executes the sequence; forward = +x, left spin increases yaw (CCW positive) — confirm sign conventions and write them in `docs/webots_findings.md`.
- [ ] **3.3 [P0] Kinematics validation.** For commands (v,ω) ∈ {(0.2,0), (0,1.0), (0.2,0.5), (−0.1,0)}: convert with `inverse_kinematics`, run for ≥3 s after settling, estimate measured v and ω from GPS/IMU finite differences, log to `logs/kinematics_test.csv`.
  *Done when:* a table of commanded vs measured v, ω with % error is saved to `docs/evidence/kinematics_validation.md` (expect < ~5–10 %; if larger, investigate wheel radius/separation/slip and fix `config.py`).
- [ ] **3.4 [P0] Odometry vs ground truth.** Implement encoder odometry in the main controller path (or in drive_test); run line, square, and arc trajectories; log odometry and GPS+IMU pose.
  *Done when:* overlay plot `docs/evidence/odometry_vs_gps.png` + final error numbers saved.
- [ ] **3.5 [P2] Pen trail:** add a `Pen` node (see `pen` sample) to draw the robot's actual path on the floor, controllable on/off. Useful for the demo visual.

---

## Phase 4 — Perception [P0]

### 4A LiDAR
- [ ] **4.1 Lidar device config** in the robot: 1 layer, 360 rays, FOV 2π, range 0.05–4.0 m, mounted at center-top. In `perception.py`, wrap: enable, `get_ranges()` (NumPy), validity mask, angles array.
  *Done when:* a test controller prints min/mean range and the number of valid rays.
- [ ] **4.2 [P0] Measure ray order/handedness.** Put an obstacle only on the robot's left (then right, then front); find which indices read short; derive and store `LIDAR_ANGLE_SIGN` / `LIDAR_ANGLE_OFFSET` in `config.py`. Document in `docs/webots_findings.md`.
  *Done when:* `ray_angle(i)` returns +90° for the left obstacle, 0° for front, verified three times.
- [ ] **4.3 [P0] Sector queries + obstacle flag.** `min_range_in_sector(scan, a_min, a_max)`, `front_blocked(scan)` (< `STOP_DIST` in ±30°), plus `lidar_to_world` hook.
  *Done when:* console prints `OBSTACLE AHEAD 0.42 m` when the robot faces a crate.
- [ ] **4.4 [P0] LiDAR validation + evidence.** Measure true vs measured at 0.5/1.0/2.0 m (use supervisor ground truth for the true distance), save table; save a scan snapshot and render `tools/plot_lidar.py` → `docs/evidence/lidar_scan.png` (world-frame hit points overlaid on the known map).
  *Done when:* error table + plot saved.

### 4B Camera & vision
- [ ] **4.5 Camera device config** (320×240, FOV ≈ 1.0 rad, front-mounted). `perception.get_frame()` → BGR `ndarray` (drop alpha). Save 3 frames to disk from different poses.
  *Done when:* saved PNGs look correct (not flipped/blue-shifted) and show the packages.
- [ ] **4.6 [P0] `detector.py` (pure OpenCV).** HSV masks for red/green/blue, morphology, contour filter, `Detection` dataclass with heuristic score, `annotate(image, detections)`.
  *Done when:* `pytest` runs the detector on the saved frames (stored in `tests/data/`) and finds the right colors and approximately correct bbox.
- [ ] **4.7 [P0] Capture an evaluation set.** Via a small controller or the main controller, save ≥30 frames per package at varied distances (0.5–3 m), angles, and 3 lighting levels (change `DirectionalLight.intensity` via world edit/supervisor). Record ground truth (which package is visible, from supervisor geometry or `Recognition`).
  *Done when:* `tests/data/` / `docs/evidence/frames/` hold the frames + a ground-truth CSV.
- [ ] **4.8 [P0] `tools/eval_detector.py`.** Compute per-color detection rate, false-positive rate, and results per lighting level. Tune `HSV_RANGES` in `config.py` until normal-lighting detection is solid; report the dim/bright results honestly (this is the "challenge" for the slides).
  *Done when:* `docs/evidence/detector_eval.md` has the table; one annotated image saved as `docs/evidence/detection_example.png`.
- [ ] **4.9 [P0] Position estimation.** Implement the bearing + LiDAR-range + pose transform from `project_summary.md` §10.8; compare with supervisor ground truth for ≥10 placements.
  *Done when:* mean/max position error (m) saved to `docs/evidence/position_estimate.md`.
- [ ] **4.10 [P2] Ground-truth cross-check using the `Recognition` node** (see `camera_recognition` sample) to automate 4.7 labeling and 4.9 truth. *(Detector must remain our own OpenCV code.)*

---

## Phase 5 — Motion control [P0]

- [ ] **5.1 `controller.py` — `WaypointFollower`** per §10.6: PID heading, speed shaping, turn-in-place threshold, waypoint/goal tolerance, `status` (`FOLLOWING`/`BLOCKED`/`REACHED`), cross-track error.
  *Done when:* pure-logic tests with a simulated unicycle model (use `forward_kinematics`/odometry equations) show the follower reaching a multi-waypoint path without Webots.
- [ ] **5.2 [P0] Live test on hand-given waypoints** (a square and an L-shape in free space) using the real robot, GPS+IMU pose, `inverse_kinematics`, motors. Log run CSV.
  *Done when:* robot reaches all waypoints within tolerance; no spinning/oscillation at waypoints.
- [ ] **5.3 [P0] Tune** kp/ki/kd, `V_MAX`, `TURN_IN_PLACE_THRESH`, `SLOWDOWN_DIST`. Record the final gains and what each change did in `docs/progress_log.md`.
  *Done when:* mean cross-track error and max overshoot are measured and recorded (`docs/evidence/following_metrics.md`) + trajectory plot planned-vs-actual saved.
- [ ] **5.4 [P0] LiDAR safety stop wired in:** forward cone < `STOP_DIST` → v = 0, status `BLOCKED`, console message. (No re-planning yet.)
  *Done when:* robot stops before a crate placed in its path and does not collide.

---

## Phase 6 — Planning ↔ control integration & FSM

- [ ] **6.1 [P0] Planner glue.** `plan_to(pose, goal_xy)` → snap, A\*, simplify, return world waypoints + info. Print: `A* path: 14 waypoints, 6.8 m, 3.1 ms, 412 expansions`.
  *Done when:* from the start pose to Zone B center returns a valid path.
- [ ] **6.2 [P0] First integration (the key mid-eval demo).** In `intellibot_controller.py`: robot plans with A\* from start to Zone B (or to a hard-coded goal), follows the waypoints in Webots; LiDAR safety stop and detector loop run live every step without breaking timing.
  *Done when:* the robot autonomously drives the planned path to the goal; log + screen recording captured → `docs/evidence/run_astar_follow.csv`, `.mp4`.
- [ ] **6.3 [P0] Overlay evidence:** plot of A\* path, the robot's actual trajectory (from log), obstacles, and goal → `docs/evidence/planned_vs_actual.png`.
- [x] **6.4 [P0] `decision_maker.py` — FSM** per §10.9 with unit tests using fake inputs (state transitions, timeouts, ERROR paths). Every transition printed.
  *Done when:* tests pass for the happy path and for "package never found → ERROR".
- [ ] **6.5 [P0] FIND_PACKAGE live:** the robot rotates, detects the P3 (blue) package, estimates its position, prints `Package P3 detected at (x, y), est. error ...`.
  *Done when:* works from at least 3 different start orientations.
- [ ] **6.6 [P0] NAVIGATE live:** choose a free **approach pose** (≈0.45 m from the package, on the side facing the robot, reachable per A\*), plan, drive, stop and face the package.
  *Done when:* robot ends within tolerance and facing the package without touching it.
- [ ] **6.7 [P0] PICK (placeholder) → PLAN_DELIVERY → DELIVER → DONE.** PICK prints `Package P3 ... Distance: 0.xx m ... PICK placeholder (attachment in final eval)`; then plan to Zone B, drive, finish with a mission report (time, path length, replans=0, collisions=N/A until the bumper exists).
  *Done when:* one complete run FIND → … → DONE is logged and recorded; `simulationQuit(0)` called at DONE when `AUTO_QUIT=True`.
- [ ] **6.8 [P1] Repeatability:** run N≥10 times with different start poses; log success/failure, time, goal error; summarize in `docs/evidence/runs_summary.md`.
- [x] **6.9 [P1] Re-plan hooks verified by test:** a unit test blocks a path cell via `add_obstacle_world`, `path_is_blocked` returns true, re-planning with A\* finds the alternate route. (Do **not** wire it live — that's the final-eval wow feature.)
- [ ] **6.10 [P2] In-sim `Display` overlay** (see `display` sample): draw grid, path, robot, state text. Strong visual if time allows; otherwise skip.
- [ ] **6.11 [P2] `LED`/state color indicator per FSM state.**

### ✂️ CUT LINE
If time is nearly out, the **minimum viable mid-eval** is: Phase 1 (all), 2, 3, 4A.1–4A.4, 4B.5–4B.8, Phase 5, 6.1–6.3, and 6.4 (FSM tests). Everything after that is a bonus. **Stop building and start Phase 7/8.**

---

## Phase 7 — Evidence, metrics, docs

- [x] **7.1 [P0] `tools/analyze_logs.py`** → reads `logs/*.csv`, produces `docs/evidence/metrics.md` covering every row of the `project_summary.md` §13 table (only the ones actually measured; mark the rest "not yet measured").
- [ ] **7.2 [P0] Evidence folder check** — confirm these exist and are legible: world screenshots, robot screenshot, LiDAR scan plot, detection example, A\* plot, planned-vs-actual plot, kinematics table, odometry plot, detector table, following metrics, ≤60 s screen recording of a full run, final console log.
- [x] **7.3 [P0] `docs/architecture.md`:** block diagram (Perception / Planner / Control → Decision Maker → Robot), data flow, and a **plain-language "how it works" paragraph per module**, with the interface between modules (inputs/outputs). Indicate Person A/B/C ownership per `project_summary.md` §14.
- [x] **7.4 [P0] `README.md`:** requirements, how to install Python deps, how to set Webots' Python command, how to regenerate the world, how to run the demo, how to run tests, repo structure, known limitations.
- [x] **7.5 [P1] `docs/qa_prep.md`:** ≥25 likely viva questions with short correct answers, e.g.: derive differential-drive kinematics; why A\* and why octile heuristic; why inflate obstacles; difference between global planning and local avoidance; how PID heading control works and what each gain does; how HSV detection works and why red needs two ranges; how a pixel becomes a world coordinate; why GPS+IMU instead of odometry and what's the limitation; what the FSM states are; what's different from a scripted route; what would change on a real robot.
- [x] **7.6 [P1] `docs/project_report.md` skeleton** (intro, architecture, methods, results with real numbers, limitations, future work) so the final report can extend it.
- [x] **7.7 [P0] Demo script** `docs/demo_script.md`: exact steps and talking points for the 5-minute presentation (open world → run → what to say at each state → fallback if live run fails: play recording).

---

## Phase 8 — Update the deck to match reality

The draft deck `IntelliBot_MidTerm_Review.pptx` contains *expected* progress statements. Make it truthful:

- [ ] **8.1 [P0] Slide 5 (Simulation Progress):** replace each description with the real achievement; add the actual screenshots/plots (world, detection with bbox, LiDAR plot, A\* plot, planned-vs-actual) — remove any card whose feature was not achieved.
- [ ] **8.2 [P0] Slide 6 (Results & Challenges):** use measured numbers (kinematics error %, detector rate, A\* optimality, cross-track error, success rate) and **real** challenges encountered (lighting results from 4.8, controller tuning notes from 5.3, timing issues, etc.).
- [ ] **8.3 [P0] Slide 7 (Team & Next Steps):** keep the A/B/C ownership; list remaining work: dynamic re-planning, real pickup, PID polish, collision counter, dashboard, comparisons.
- [ ] **8.4 [P1] Add 1 short embedded GIF/video link or QR to the run recording** (or prepare the file for live playback).
- [ ] **8.5 [P0] Rehearsal notes:** every member can explain *every* slide; assign who presents which (A: world/robot/control; B: perception; C: planning/FSM/integration + next steps).

---

## Phase 9 — Final-eval backlog (DO NOT build now; for planning only)

| Item | Owner | Reuse from samples |
|---|---|---|
| Dynamic obstacle: spawn/move `DYN_OBSTACLE` into route at trigger time (supervisor) | C (+A) | `sample_supervisor`, `gps_supervisor` |
| LiDAR → dynamic layer → `path_is_blocked` → re-plan → continue (live) | C + B | `lidar` |
| Package pickup: Connector-based or supervisor "follow-pose" attachment; carry to zone and release | A | `connector`, `vacuum_gripper` |
| Bumper/TouchSensor collision counter | A | `bumper` |
| PID tuning with step-response analysis, speed profile / smoother trajectory | A | `motor` |
| Detector robustness (lighting, partial occlusion), obstacle classification | B | `camera_recognition`, `camera_segmentation` |
| In-sim dashboard (map, path, state, replans) | all | `display`, `display_supervisor` |
| Mission-complete report with real replans/collisions | C | — |
| Optional: Dijkstra/RRT comparison, second robot, YOLO | any | `emitter_receiver` |

---

## Final acceptance (mid-eval)

- [ ] `pytest` all green
- [ ] One full autonomous run recorded: FIND_PACKAGE → NAVIGATE → PICK(placeholder) → PLAN_DELIVERY → DELIVER → DONE with mission report
- [ ] All P0 metrics measured and saved under `docs/evidence/`
- [ ] Deck matches the evidence (no unproven claims)
- [ ] README + architecture + demo script + Q&A prep written
- [ ] Every module has a short "how to explain it" paragraph for its future human owner
