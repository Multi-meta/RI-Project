# CHECKLIST_V2.md — IntelliBot: complete build plan (Mid-eval → Final completion)

> **Supersedes `checklist.md`.** Read in this order: `project_summary.md` (conventions & v1 spec) → `project_summary_v2_addendum.md` (full-scope specs; **overrides** the summary where they conflict) → this file.
>
> **How to work:** top to bottom. Tick `- [x]` only when the **Done when** check passes. Log 1–3 lines per task in `docs/progress_log.md`. Never fake results; if you cannot run Webots, give the human exact steps and wait for output.
>
> **Priority tags:** `[P0]` required · `[P1]` should-have · `[P2]` nice-to-have.
> **Tiers (global cut lines):**
> - **Tier 1 — Final-eval core:** live dynamic re-planning, EKF fusion with lights-off demo, sensorized arm pick&place onto shelf slots (single robot), 2 robots with basic traffic rules, rear sensors + bumpers, mission report.
> - **Tier 2 — Strong final:** moving-obstacle tracking + DWA + rear-approach avoidance, pile with ArUco IDs, 3rd robot, reservation-based traffic, in-sim dashboard.
> - **Tier 3 — Polish/extras:** space-time planning, re-handling buried boxes, radar, web dashboard, Dijkstra/RRT comparison.
> Finish a tier's items before starting the next. **Feature freeze gate** is in Phase 17.
>
> **Part I (Phases 0–8)** = mid-evaluation foundation. **If a Part I task is already done and its evidence exists, tick it and move on.** Part I constants (robot size, world size) will be **parametrized and upgraded** in Part II (Phases 9, 13, 14) — keep everything in `config.py`.

---

# PART I — FOUNDATION (mid-evaluation)

## Phase 0 — Setup & reconnaissance
- [x] **0.1 [P0] Environment check** → `docs/webots_findings.md`: Webots version, OS, Python used by Webots; `numpy`, `cv2`, `matplotlib` import in a controller. *Done when:* a throwaway controller prints their versions.
- [x] **0.2 [P0] Repo skeleton** per `project_summary.md` §9 (+ new modules from the addendum listed in Phase 9). Do not edit existing sample folders. *Done when:* `pytest` runs with no errors.
- [x] **0.3 [P0] Samples audit** → `docs/samples_audit.md`. Open the `.py` in: `lidar`, `camera`, `camera_recognition`, `gps`, `inertial_unit`, `position_sensor`/`encoders`, `motor`, `distance_sensor`, `sample_supervisor`, `gps_supervisor`, `connector`, `vacuum_gripper`, `display`, `pen`, `emitter_receiver`, plus (needed later) `light_sensor`, `force_sensor`, `force3d_sensor`, `linear_motor`, `range_finder`, `led`, `speaker`, touch/bumper sample if present. *Done when:* each has the exact API calls to reuse.
- [x] **0.4 [P1]** Open 2–3 sample worlds (`projects/samples/devices/worlds/`) to copy node field settings (Lidar, Camera, DistanceSensor types, LightSensor, TouchSensor types, Emitter/Receiver).
- [x] **0.5 [P0] World header template** for the installed version (`#VRML_SIM`, `coordinateSystem "ENU"`, `EXTERNPROTO` needs) → `tools/world_header.txt`. *Done when:* a trivial generated world opens cleanly.

## Phase 1 — Pure-Python core [P0]
- [x] **1.1 `config.py`** — all constants (robot, motors, devices, grid, thresholds, HSV, layout, task, tolerances). Make it **parametrizable** (dict/dataclass; `NUM_ROBOTS`, `ROBOT_ID` ready).
- [x] **1.2 `kinematics.py` + tests** — forward/inverse, clamping (preserve curvature), `wrap_to_pi`, `Odometry`.
- [x] **1.3 `pid.py` + tests** — limits, anti-windup, reset.
- [x] **1.4 `mapping.py` + tests** — `OccupancyGrid`, transforms, circular inflation, `add_obstacle_world`, `clear_dynamic`, `path_is_blocked`, `lidar_to_world`.
- [x] **1.5 `a_star.py` + tests** — 8-connected, no corner cutting, octile; cost equals Dijkstra on ≥50 random grids.
- [x] **1.6 `path_utils.py` + tests** — Bresenham simplification, resampling/smoothing, grid→world, length.
- [x] **1.7 `tools/plot_astar.py`** → `docs/evidence/astar_*.png`. *Done when:* figure is slide-ready.
- [x] **1.8 [P0]** Warehouse grid test: path exists receiving→zone; **≥2 distinct routes** (block one, path still exists).

## Phase 2 — Warehouse world v1 [P0]
- [x] **2.1 `tools/generate_world.py`** from `config.py` (arena, shelves, crates, pickup area, packages, zones, flat lighting, `basicTimeStep` 16). *Done when:* opens with zero console warnings.
- [x] **2.2** Screenshots → `docs/evidence/world_*.png`.
- [x] **2.3 [P1]** Startup world-vs-config consistency check via Supervisor (`getFromDef`).

## Phase 3 — Robot v1, locomotion, kinematics validation [P0]
- [x] **3.1** Robot node (body, wheels w/ HingeJoint+RotationalMotor+PositionSensor, zero-friction casters, Lidar, Camera, GPS, InertialUnit; `supervisor TRUE`). *Done when:* stable at rest for 10 s.
- [x] **3.2** `controllers/drive_test/` scripted motions; confirm sign conventions → `docs/webots_findings.md`.
- [x] **3.3** Kinematics validation table (commanded vs measured v, ω) → `docs/evidence/kinematics_validation.md`.
- [x] **3.4** Odometry vs GPS+IMU on line/square/arc → `docs/evidence/odometry_vs_gps.png`.
- [x] **3.5 [P2]** `Pen` trail.

## Phase 4 — Perception v1 [P0]
- [x] **4.1–4.4 LiDAR:** wrapper; **measure ray order/handedness**; sector queries + obstacle flag; validation table + scan plot.
- [x] **4.5–4.8 Camera/vision:** BGR frames; `detector.py` (HSV, morphology, contours, heuristic score); evaluation set (distance/angle/3 lighting levels); `eval_detector.py` table; tune thresholds.
- [x] **4.9** Package position estimate (bearing + LiDAR range + pose) with error vs ground truth.
- [ ] **4.10 [P2]** `Recognition` node as automatic ground truth.

## Phase 5 — Motion control v1 [P0]
- [ ] **5.1** `WaypointFollower` (PID heading, speed shaping, turn-in-place, status) + pure-logic tests with a simulated unicycle.
- [ ] **5.2** Live test on hand-given waypoints. **5.3** Tune + metrics + planned-vs-actual plot. **5.4** LiDAR safety stop.

## Phase 6 — Integration v1 & FSM v1
- [x] **6.1 [P0]** Planner glue (`plan_to`).
- [ ] **6.2 [P0]** Robot drives an A* path in Webots.
- [x] **6.3 [P0]** Overlay plot.
- [x] **6.4 [P0]** FSM v1 (IDLE→FIND_PACKAGE→NAVIGATE→PICK(placeholder)→PLAN_DELIVERY→DELIVER→DONE/ERROR) + unit tests.
- [ ] **6.5–6.7 [P0]** Live FIND/NAVIGATE/…/DONE with mission report.
- [x] **6.8 [P1]** ≥10 repeatability runs.
- [x] **6.9 [P1]** Re-plan unit test.
- [ ] **6.10 [P2]** Display overlay.
- [ ] **6.11 [P0] Tag the mid-eval baseline in git (`v1-mid-eval`)** and archive its evidence (`docs/evidence/v1/`). This is the **ablation baseline** for Part II.

## Phase 7 — Evidence & docs (mid) / Phase 8 — Deck
- [x] **7.1 [P0]** `analyze_logs.py`.
- [ ] **7.2 [P0]** Evidence folder check.
- [x] **7.3–7.7 [P0/P1]** `architecture.md`, `README.md`, `qa_prep.md`, `project_report.md` skeleton, `demo_script.md`.
- [x] **8.1–8.3 [P0]** Mid deck matches real evidence; simulation progress, results/challenges, team ownership, and next steps updated.
- [ ] **8.4 [P1]** Add a short embedded GIF/video link or QR to the run recording.
- [x] **8.5 [P0]** Rehearsal notes and presentation ownership.

---

# PART II — FINAL EVALUATION (everything below is new scope)

## Phase 9 — Architecture upgrade for v2 (prep) [P0]
- [ ] **9.1 Refactor interfaces.** Define typed data classes (dataclasses) with `timestamp`, `valid`, `variance/quality`: `SensorReading`, `Pose2D(+cov)`, `ObstacleReport`, `Track`, `Detection`, `Task`, `Message`. *Done when:* modules exchange only these types; tests updated.
- [ ] **9.2 Per-robot identity.** Controller reads `ROBOT_ID` from `controllerArgs`/`getName()`; per-robot log files `logs/<run>_<id>.csv`; no module-level global state. *Done when:* two instances can run in one process-per-robot without clashing.
- [ ] **9.3 Create new module files (with docstrings + tests dir entries):** `sensor_models.py`, `fault_injector.py`, `sensor_health.py`, `ekf_localization.py`, `obstacle_fusion.py`, `tracker.py`, `costmap.py`, `dwa.py`, `rear_safety.py`, `arm_kinematics.py`, `trajectory.py`, `manipulation.py` (grasp/place FSMs), `inventory.py`, `fleet_manager.py` (separate controller folder `controllers/fleet_manager/`), `comms.py`, `traffic.py`, `dashboard.py`.
- [ ] **9.4 Message bus abstraction (`comms.py`)** with a pure-Python in-memory implementation (for tests) and an Emitter/Receiver implementation (Webots). JSON encode/decode, channel handling, drop/range simulation hook.
  *Done when:* unit tests send/receive/drop messages through the in-memory bus.
- [ ] **9.5 `config.py` v2:** `NUM_ROBOTS`, robot v2 dims, sensor suite spec (names, positions, σ), fault schedule, moving-obstacle lanes/params, shelf/slot geometry, pile layout, ArUco IDs, comms params, traffic params, random seed.
- [ ] **9.6 [P0]** Add a **`scenario` runner** concept: a config/JSON describing a scenario (robots, tasks, faults, obstacles, duration, seed) so experiments are reproducible.

## Phase 10 — Sensor suite expansion, noise & fault models [Tier 1]
- [ ] **10.1 [P0] Sensor models (`sensor_models.py`).** Noise injection per sensor (σ in config); wrapper API returns `SensorReading`. Use Webots node noise fields where available, software noise otherwise. *Done when:* histograms/stats of noisy vs clean data match configured σ.
- [ ] **10.2 [P0] Add devices to Robot** (extend generator): rear camera; IR ring (FL, FR, L, R, RL, RR + front/rear); ultrasonic (sonar-type DistanceSensor) front/rear (+sides); `LightSensor`; front and rear `TouchSensor` bumpers; GPS with `accuracy`; keep names in `config.py`. *Done when:* each sensor prints plausible readings in front of a known obstacle (verify type, lookup table, beam angle).
- [ ] **10.3 [P0] Coverage table** `docs/sensor_coverage.md` + test: front/left/right/rear each have ≥2 independent modalities (assert from config).
- [ ] **10.4 [P0] Fault injector (`fault_injector.py`)**: scheduled/triggered faults — lights-off (Supervisor toggles `DEF LIGHT_*` `on`/`intensity`), LiDAR dropout/frozen, GPS dropout, camera blind, stuck-at/bias. Faults must be applied by the **Fleet Manager/Supervisor**, not visible to robot logic except through sensor behavior.
  *Done when:* console shows `FAULT lights_off ACTIVE`, the camera image visibly goes dark, the light sensor reading drops.
- [ ] **10.5 [P0] Characterization:** per sensor measure range accuracy/noise/latency vs distance (0.2–4 m) and under faults → `docs/evidence/sensor_characterization.md` (table + 1 plot).
- [ ] **10.6 [P1] Calibration of mounting transforms** (sensor positions in robot frame) stored in `config.py`; verify with known targets.

## Phase 11 — Kalman filtering & sensor fusion [Tier 1; extensions Tier 2]
- [ ] **11.1 [P0] `ekf_localization.py` + tests** (pure): predict from odometry increments; updates for IMU yaw and GPS/beacon; Joseph form; angle-wrapped innovations; gating (χ²). *Done when:* tests on simulated trajectories show RMSE lower than odometry-only and than GPS-only; consistency (NEES) within bounds.
- [ ] **11.2 [P0] Landmark updates (range-bearing).** Detect known landmark posts from LiDAR clusters (`landmarks.py`); add EKF update with Jacobian. *Done when:* with GPS disabled, pose error stays bounded (vs odometry-only drift) in test and in Webots.
- [ ] **11.3 [P0] `sensor_health.py` + tests:** staleness, validity, plausibility, camera brightness/contrast quality, NIS tracking → `OK/DEGRADED/FAILED` + `trust`; R inflation policy; logs transitions.
- [ ] **11.4 [P0] Wire EKF into the robot** (replace GPS+IMU as operating pose; keep ground truth only for logging). Log `x_est, P diag, x_true, health` each step.
  *Done when:* robot completes the v1 mission using the **EKF pose**, with RMSE logged.
- [ ] **11.5 [P0] Camera landmark/marker update (optional modality)** with R scaled by light level so that **lights-off** de-weights it automatically.
- [ ] **11.6 [P0] `obstacle_fusion.py` + tests:** per-sector inverse-variance fusion across LiDAR/IR/sonar (+tracks/comm data when available), `VERIFIED/UNVERIFIED/CONFLICT`, trust-weighted, outputs `ObstacleReport`. Safety stop and costmap consume it.
  *Done when:* tests: two agreeing sensors → VERIFIED; one dropout → still detected, UNVERIFIED; strong disagreement → CONFLICT + speed cap.
- [ ] **11.7 [P0] Fault experiments (the "lights go off" demo):** scenarios **S3 lights-off**, **S4 LiDAR dropout**, **S4b GPS dropout**. For each: log which sensors were trusted, speed cap, and whether navigation continued safely; produce RMSE table **with vs without fusion** and a timeline plot of health/trust → `docs/evidence/fusion_results.md`, `fusion_timeline.png`.
- [ ] **11.8 [P1] Tune Q/R** using ground-truth NEES; keep a tuning log (`docs/ekf_tuning.md`).
- [ ] **11.9 [P1] Fusion in manipulation checks** (see 14.x): "box present"/"slot free" decisions combine force, width, palm IR, camera.
- [ ] **11.10 [P2] Map-matching/ICP scan-to-map update** as an extra localization source.
- [ ] **11.11 [P1]** Document fusion as: inverse-variance weights + gating + health + verification (not a "hard switch"). Add to `architecture.md` and `qa_prep.md` (derive Kalman gain, why gating, what NEES means).

## Phase 12 — Dynamic obstacles & advanced navigation [Tier 1: 12.1–12.4; Tier 2: rest]
- [ ] **12.1 [P0] Live dynamic re-planning (the original "wow" feature).** LiDAR hits not in the static map → dynamic layer → `path_is_blocked` → A* re-plan → continue. Console: `OBSTACLE DETECTED → path BLOCKED → replanning → new path found`. Place a **static pop-up box** in an aisle via Supervisor at a trigger time.
  *Done when:* robot reroutes via the second route and completes the mission; replan count + reason logged.
- [ ] **12.2 [P0] `costmap.py` + tests:** layers (static, inflation with gradient, dynamic, predicted-dynamic, reservation) and clear logic (clear cells observed free).
- [ ] **12.3 [P0] Recovery behaviors:** wait → rotate → back up (**only if rear verified clear**) → replan with inflated costs → ERROR; configurable timeouts; unit-tested FSM logic.
- [ ] **12.4 [P0] Bumper collision counter + mission KPIs** (collisions, min clearance) added to logs.
- [ ] **12.5 [P0] Moving obstacles in the world:** `DEF CART_*` (and `WORKER_*`) Solids with physics moved by the Supervisor with `setVelocity` along lanes (patrol, cross-aisle at scheduled time, stop-and-go, pop-up). Parameters + seed in config. *Done when:* carts move physically, push/ collide properly, visible in all scenarios.
- [ ] **12.6 [P0] `tracker.py` + tests:** clustering, association, constant-velocity Kalman tracks, static/moving classification, prediction, TTC. *Done when:* tests recover velocity within tolerance on synthetic data; in Webots the tracker reports a cart's velocity within ~0.1 m/s.
- [ ] **12.7 [P0] Predicted-dynamic layer:** swept-area write to costmap; global replan avoids predicted crossings; log `TTC`.
- [ ] **12.8 [P0] `dwa.py` + tests:** dynamic window, admissibility, scoring, reverse only if rear clear; integrates with the follower (global waypoints → DWA local goal). *Done when:* in a test world with moving obstacles the robot avoids collisions in ≥ 90 % of 20 runs (record real number).
- [ ] **12.9 [P0] Rear sensing & `rear_safety.py`:** rear camera + rear IR/sonar + LiDAR rear sector → rear-approach detection (closing speed, TTC); rules: no reverse unless clear; warn/yield on `TTC < thresholds`; LED/log/broadcast; optional audible warning via `Speaker`.
  *Done when:* **S6 rear-approach**: another robot/cart approaches from behind; the robot detects it (log shows which sensors), warns, and yields or moves aside without collision.
- [ ] **12.10 [P1] Speed governor** (`v_cap` from nearest-object distance, sensor health, zone).
- [ ] **12.11 [P1] Local-minimum / stuck detection** and recovery.
- [ ] **12.12 [P2] D\* Lite or incremental re-planning; timing comparison with full A\*.**
- [ ] **12.13 [P0] Navigation experiments:** N≥20 runs per scenario (static pop-up, crossing cart, stop-and-go, rear approach): success, collisions, replans, clearance, time → `docs/evidence/navigation_results.md` + plot.

## Phase 13 — Warehouse v2: shelves, pile, inventory [Tier 1 core; ArUco/pile detail Tier 2]
- [ ] **13.1 [P0] Update `config.py` + `generate_world.py` for World v2** (§A7): 12×8 m arena, shelf units with 3 levels × 3 bays (slot IDs, heights, interior size, back wall), receiving **pile** (2 layers, 9–12 boxes, IDs), dispatch zones A/B, 3 robot docks, landmarks, multiple lights (`DEF LIGHT_*`), moving-obstacle lanes. *Done when:* world opens with no warnings; grid rebuilt from config; aisles ≥ 1.6 m (or one-way) verified by a test; ≥2 routes receiving→any shelf.
- [ ] **13.2 [P0] Box physics:** mass 0.3–0.5 kg, friction material (`ContactProperties`), stable stacking (boxes don't jitter/slide in the pile). *Done when:* pile stays stable for 60 s of simulation.
- [ ] **13.3 [P0] ArUco markers:** generate marker textures (`cv2.aruco`), apply as `ImageTexture` on box faces and slot tags; `aruco_detect.py` returns ID + pose (intrinsics from FOV). *Done when:* IDs and distances are read correctly at 0.5–1.5 m in the rendered camera image (record the working size/resolution); fallback documented (A13).
- [ ] **13.4 [P0] `inventory.py` + tests:** slot occupancy, package metadata, reserve/assign/confirm/release, assignment policy (heavy→low, same category grouped, nearest free, reserved while carried). Property tests: no double assignment.
- [ ] **13.5 [P1] Pile model for planning:** list of boxes with layer/position/accessibility (front face free, top layer first); `select_box(task)` returns an accessible matching box or `None`.
- [ ] **13.6 [P1] Dock pose computation:** for each bay/pile position compute pre-dock and dock poses (standoff, heading) from geometry; test reachability with A* on the inflated grid.
- [ ] **13.7 [P1]** World screenshots + a short annotated diagram of zones → `docs/evidence/world_v2_*.png`.

## Phase 14 — Mobile manipulator (Robot v2) [Tier 1 core; refinement Tier 2]
- [ ] **14.1 [P0] Chassis v2** in the generator: dims/mass/wheels/casters per §A9.1; re-validate kinematics (rerun 3.3) and re-tune the follower; update inflation radius in config; arm **stowed** inside footprint. *Done when:* robot drives all routes without tipping, with stowed arm.
- [ ] **14.2 [P0] Lift + 2R arm + wrist + gripper models** (LinearMotor lift, RotationalMotors, finger LinearMotors, PositionSensors on all joints, joint limits, `maxForce/maxTorque`, damping). *Done when:* each joint tracks position commands in a joint test controller without oscillation; arm weight doesn't destabilize the base.
- [ ] **14.3 [P0] `arm_kinematics.py` + tests:** FK, analytic IK (both elbow solutions, limits, out-of-reach rejection). *Done when:* IK→FK round-trip error < 1e-6; workspace test confirms **every shelf slot and every pile position is reachable** from the planned dock poses (table saved).
- [ ] **14.4 [P0] `trajectory.py` + tests:** quintic joint trajectories; velocity/acceleration limits; Cartesian waypoint → IK → joint trajectory. *Done when:* test shows zero start/end velocity/acceleration; live motion is smooth.
- [ ] **14.5 [P0] Gripper sensing:** finger `TouchSensor` (force type), finger position, palm IR `DistanceSensor`, (optional) hand camera. Calibration: force vs closing, width vs finger position. *Done when:* `grasp_state()` returns {EMPTY, CONTACT, HOLDING(width, force), SLIPPING}.
- [ ] **14.6 [P0] Pick from floor/pile (`manipulation.py` — `PickFSM`)** per §A9.4: align, open, approach, advance with palm IR, close until force threshold, verify width/force, lift 5 cm check, retract, stow. Failure branches and retry limits implemented and unit-tested with fake sensors.
  *Done when:* single robot picks **a floor box** successfully in ≥ 8/10 trials (record the real rate and failures).
- [ ] **14.7 [P0] Docking to shelf bay:** pre-dock → fine alignment using LiDAR line-fit (parallel + distance) and slot ArUco tag/sonar/IR; tolerance check ±3 cm lateral (the 2R arm absorbs small lateral error), ±3° heading. *Done when:* dock success measured over 10 trials; error stats saved.
- [ ] **14.8 [P0] Place onto shelf slot (`PlaceFSM`)** per §A9.5 (lift to level + clearance, slot-free check, extend, lower, open, verify release, retract, stow, confirm). *Done when:* places a box on **each of the 3 levels** with position error recorded; slot occupancy confirmed by sensors, inventory updated.
- [ ] **14.9 [P0] Pick from the pile:** select accessible box (top layer first), read ArUco ID/pose, compute grasp pose through the transform chain `T_world_robot · T_robot_cam · T_cam_marker`, pick. *Done when:* picks a requested ID from the pile in ≥ 7/10 trials; transform-chain accuracy (m) logged.
- [ ] **14.10 [P0] Retrieve from shelf slot** (reverse of place) and **drop at dispatch zone** (lower, open, retract, verify).
- [ ] **14.11 [P1] Grasp failure analysis:** log categories (missed, slip, collision, unreachable, vision fail) and fixes; one table in the report.
- [ ] **14.12 [P1] Manipulation under fault:** lights-off during pick/place → camera-dependent steps use fallback (known coordinates + palm IR + force) or pause safely; demonstrate once.
- [ ] **14.13 [P2] Re-handling:** if the target is buried, move blockers to a buffer, then pick the target.
- [ ] **14.14 [FALLBACK] If 14.6–14.8 are unreliable after a bounded effort (set a time-box):** switch to Cartesian arm → then `VacuumGripper`/`Connector`; keep all verification logic; document as a simplification (never silently teleport a box; if a supervisor "attach" is used, state it explicitly in the report).

## Phase 15 — Multi-robot (2–3 robots) [Tier 1: 2 robots basic; Tier 2: 3 robots + reservations]
- [ ] **15.1 [P0] Spawn N robots** (config `NUM_ROBOTS`; unique names/IDs, docks, `controllerArgs`, unique color/marker on back and sides). *Done when:* 2 robots run the same controller independently in one world without crashes/physics issues.
- [ ] **15.2 [P0] `controllers/fleet_manager/`**: Supervisor controller that owns task queue + inventory, runs fault injector and moving obstacles, logs ground truth. *Done when:* it logs all robot poses and can assign a task.
- [ ] **15.3 [P0] Comms (`comms.py` over Emitter/Receiver):** heartbeat (pose, velocity, state, path ahead, health), task messages, slot messages; configurable range/drop. *Done when:* each robot sees the others' heartbeats; message loss test passes.
- [ ] **15.4 [P0] Task generation & allocation:** STORE tasks (pile → slot) and RETRIEVE tasks (slot → zone); greedy nearest-feasible allocation; one robot per package/slot; timeouts + reassignment on failure. *Done when:* 2 robots complete a batch of ≥6 tasks without duplicate assignments.
- [ ] **15.5 [P0] Traffic rules v1:** one-way aisles + right-of-way + yield at intersections; other robots treated as dynamic obstacles (comm poses + sensed tracks, cross-verified). *Done when:* two robots cross an intersection and pass in an aisle without collision in ≥ 20 runs.
- [ ] **15.6 [P0] Rear/side robot avoidance:** other robots identified by markers; rear-approach logic from 12.9 applies to robots. *Done when:* scenario **S6** works with a robot approaching from behind.
- [ ] **15.7 [P1] Reservation-based traffic (`traffic.py`):** corridor/cell reservations with time windows; space-time A* with short horizon; conflicts logged. *Done when:* tests show conflict-free schedules for scripted crossing cases.
- [ ] **15.8 [P1] Deadlock detection & resolution** (mutual wait > T → priority back-off/alternate route). *Done when:* an engineered head-on aisle case resolves automatically.
- [ ] **15.9 [P1] Comm-loss mode (S7):** drop all messages for T s; robots slow down, rely on local sensing, don't reserve; recover after. *Done when:* no collision and mission eventually completes.
- [ ] **15.10 [P1] Third robot** + re-run traffic tests; performance check (real-time factor).
- [ ] **15.11 [P1] Throughput experiment:** 1 vs 2 vs 3 robots, same task batch, N≥5 runs each → throughput, wait time, conflicts, speed-up → `docs/evidence/multirobot_results.md` + plot.
- [ ] **15.12 [P2]** Auction/Hungarian allocation and comparison against greedy; battery/charging model.

## Phase 16 — Full mission integration & scenario suite [Tier 1]
- [ ] **16.1 [P0] Robot FSM v2** (§A11) with NAV sub-states, SAFE_STOP/DEGRADED overrides, STORE/RETRIEVE flows, full logging; unit tests with fake inputs for all transitions incl. failure paths.
- [ ] **16.2 [P0] Single-robot full mission (S1):** pile → pick → navigate → dock → place on assigned slot → confirm → report. *Done when:* ≥ 8/10 end-to-end successes (record real rate).
- [ ] **16.3 [P0] Retrieve mission (S2):** shelf slot → dispatch zone.
- [ ] **16.4 [P0] Scenario suite (each scripted via the scenario runner, each with expected-outcome checks and log):**
  - S1 inbound store, nominal · S2 outbound retrieve
  - S3 lights-off mid-mission · S4 LiDAR dropout · S4b GPS dropout
  - S5 moving cart crossing aisle (tracking + predictive avoid)
  - S6 rear approach by robot/cart
  - S7 comm loss (multi-robot)
  - S8 stress: 3 robots, mixed STORE/RETRIEVE batch, moving obstacles, one fault
- [ ] **16.5 [P0] Mission report generator:** per robot and fleet: tasks done/failed, time, path length, replans (+reasons), collisions, min clearance, grasp/place stats, sensor-health events, EKF RMSE, throughput.
- [ ] **16.6 [P1] Reproducibility:** seeds, scenario JSON saved with each log, `tools/run_scenario.py` (CLI: scenario name → runs Webots in batch/fast mode, collects logs, runs analysis).
- [ ] **16.7 [P1] Real-time/performance profiling;** fix hotspots (planning rate, tracker, DWA sampling, image processing resolution).

## Phase 17 — Dashboard, visualization, demo assets [Tier 2]
- [ ] **17.1 [P1] In-sim dashboard** (`Display` on Fleet Manager): map with robots, paths, tracks, costmap overlay toggle, inventory grid (slot occupancy), task queue, sensor health indicators, replan counter. Reuse `display`/`display_supervisor` sample patterns.
- [ ] **17.2 [P1]** Per-robot `Pen` trails + LED color = FSM state; clear on/off toggle.
- [ ] **17.3 [P1]** Analysis plots (auto-generated): EKF vs truth, health timeline, track velocities, trajectories per robot, throughput bars, grasp success breakdown.
- [ ] **17.4 [P2]** Optional web dashboard (reads log stream; not required).
- [ ] **17.5 [P0] ✂️ FEATURE FREEZE GATE.** Stop adding features. Remaining time → stabilization, evidence, docs. Record in `progress_log.md` which tiers were completed and which items were dropped (and why).

## Phase 18 — Evaluation & ablations (evidence for the final report) [P0]
- [ ] **18.1 Experiment matrix** (`docs/experiments.md`): scenario × configuration × N runs × seeds. Configurations: **baseline v1** · +EKF fusion · +tracker/DWA · +rear safety · +multi-robot.
- [ ] **18.2 Run the matrix**; store raw logs under `logs/final/`.
- [ ] **18.3 Analysis:** tables with mean ± std (and success counts) for all groups in addendum §A12; **ablation table** showing the value of each feature (e.g., RMSE with/without fusion under lights-off; collisions with/without tracker; throughput 1/2/3 robots).
- [ ] **18.4 Failure analysis:** list the real failure modes observed (grasp, docking, comms, perception, deadlock) with counts and mitigations. Honest limitations section.
- [ ] **18.5 Statistical sanity:** no cherry-picking; report all runs or state exclusion rules.

## Phase 19 — Documentation, demo, viva readiness [P0]
- [ ] **19.1 Code quality:** docstrings, remove dead code, `pytest` green, `run_all_tests` script, lint pass, `requirements.txt` final, config documented.
- [ ] **19.2 `README.md` final:** setup (Python deps + Webots Python command), world generation, run scenarios, run tests, repo map, results summary, limitations.
- [ ] **19.3 `docs/architecture.md` final:** v2 block diagram (perception+fusion / planning+traffic / control+manipulation / fleet manager), data flow, message protocol, module "how it works" paragraphs with owner A/B/C.
- [ ] **19.4 `docs/project_report.md`:** intro, related work, system design, methods (kinematics, EKF, tracker, DWA, A*, IK, trajectory, traffic), experiments, results, discussion, limitations, future work (SLAM, learning-based grasping, real hardware).
- [ ] **19.5 Demo script (`docs/demo_script.md`) + recorded video(s):** (a) normal pick-store with arm, (b) lights-off fusion demo, (c) moving obstacle + re-plan, (d) rear approach, (e) 3-robot batch + dashboard. Keep a **backup recording** in case the live run fails.
- [ ] **19.6 Final presentation deck:** problem, novelty, architecture, sensors & fusion (formulas), manipulation, multi-robot, results with real numbers, limitations, future work, team contributions. Only measured claims.
- [ ] **19.7 Viva prep (`docs/qa_prep.md` v2):** ≥ 50 Q&A covering: kinematics & odometry; EKF derivation, Q/R meaning, gating, NEES; why multiple sensors & what failed when lights went off; how verification works; tracker/Kalman CV model; DWA formulas; A* vs D* Lite; costmap layers; IK derivation & workspace; grasp verification logic; transforms (world↔robot↔camera↔marker); multi-robot allocation & deadlock handling; limitations vs real robots. Each member rehearses their own + one cross-module question.
- [ ] **19.8 Ownership check:** every module has a named owner (A/B/C) and a "how to explain it" paragraph; each member can run any scenario from the README.

---

## Milestone gates (use to track overall progress)

| Gate | Condition |
|---|---|
| **M1** (Mid-eval baseline) | Part I complete, `v1-mid-eval` tagged, evidence archived |
| **M2** | v2 architecture + sensor suite + fault injection working (Phases 9–10) |
| **M3** | EKF + health + obstacle fusion; lights-off demo with RMSE table (Phase 11) |
| **M4** | Live re-planning + moving obstacles + tracker + DWA + rear-safety scenario (Phase 12) |
| **M5** | World v2 + inventory + ArUco pile/shelves (Phase 13) |
| **M6** | Single robot pick → dock → place on shelf slots (Phase 14) |
| **M7** | 2 robots complete a task batch with traffic rules (Phase 15 core) |
| **M8** | Full scenario suite S1–S8 runs; mission reports generated (Phase 16) |
| **M9** | Feature freeze; experiments + ablations done (Phases 17–18) |
| **M10** | Report, deck, videos, viva prep complete (Phase 19) |

## Suggested parallel split for the 3 humans (Part II)

| Phase | A — Robot/Manipulation/Control | B — Perception/Fusion/Tracking | C — Planning/Fleet/Integration |
|---|---|---|---|
| 9 | chassis/config for arm | data classes for sensors | message bus, scenario runner |
| 10 | bumpers, joint sensors | sensor models, IR/sonar/light/rear cam, fault injector | light/obstacle control hooks in Fleet Manager |
| 11 | actuator-side tests | **EKF, health, obstacle fusion**, fault experiments | consume fused pose/obstacles in nav |
| 12 | speed governor/follower with new chassis | **tracker, rear perception** | **costmap, replanning, DWA, recovery, moving obstacles** |
| 13 | box/shelf physics, slot geometry | ArUco generation/detection | inventory, world generator, dock poses |
| 14 | **lift/arm/gripper, IK, trajectory, pick/place** | gripper sensing, pile perception, transform chain | pick/place integration into FSM |
| 15 | rear/side behaviors on robot | cross-verification with comm poses | **Fleet Manager, allocation, traffic, deadlocks** |
| 16–19 | manipulation stats, demos | fusion/tracking stats, plots | integration, dashboard, scenario runs, report assembly |

## Final acceptance
- [ ] `pytest` all green; scenarios S1–S8 run from `tools/run_scenario.py`
- [ ] Lights-off scenario: navigation continues on remaining sensors; RMSE (fused vs unfused) reported
- [ ] Arm picks from a pile and places on shelf slots with sensor-verified results; success rates measured
- [ ] 2–3 robots complete a batch concurrently; moving obstacles avoided; rear-approach handled; collisions counted
- [ ] Ablation table + failure analysis + honest limitations written
- [ ] Report, deck, demo script, videos, Q&A prep complete; every claim backed by logs in `docs/evidence/` or `logs/final/`
