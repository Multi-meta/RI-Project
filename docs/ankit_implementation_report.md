# Ankit — implementation report: EKF navigation, dynamic replanning, safety, manipulation robustness

Branch `fix/ankit-ekf-navigation-manipulation` (from `main` 99b2cf1). Everything below is
implemented and tested **offline** (unit tests + the kinematic mission simulator). None of it
has been run in Webots yet; section 10 lists exactly what a teammate with Webots must check.

Evidence labels used here: **[unit]** pytest on pure logic; **[kinematic]**
`tools/offline_mission_sim.py` (real FSM / EKF / planner / arm code, unicycle base, ray-cast
LiDAR, no physics); **[Webots pending]** not run in Webots.

---

## 1. Root causes found

| Reported symptom | Root cause in the code | Evidence |
|---|---|---|
| After a collision or after being moved, the robot replans from where it **used to be** and drives back to the old route | `intellibot_controller.py` fed the EKF `ds = hypot(dx, dy)` of two odometry poses (always ≥ 0, so reversing was predicted as forward motion) and an **unwrapped** `dtheta` (±2π jump at the ±π seam). Worse, the χ² gate rejects every GPS fix that disagrees with the prediction and nothing ever recovers: once the wheels missed a displacement (bump, push, manual drag in Webots) **every later fix was rejected** and the robot kept navigating from the stale pose. The EKF was also seeded with a fallback constant before the first valid GPS sample. | [unit] `test_without_kidnap_recovery_the_filter_stays_lost` reproduces it; [kinematic] ablation D |
| Replanning after an obstacle sends the robot somewhere odd / back towards the obstacle | (a) on a safety stop the robot's own cell lies inside the inflated obstacle, and A* "snaps" a blocked start to **any** free cell up to 2 m away — possibly behind the obstacle or back on the old route; (b) only **one LiDAR ray point** was marked (radius 0.12 m), so the replanned route often grazed the real obstacle; (c) nothing ever cleared a marked cell (ghosts) and nothing detected an obstacle before the 0.30 m stop | code reading + [unit] `test_safe_start_*`, `test_static_obstacle_on_route_replans_before_the_safety_stop` |
| Robot stuck facing a wall after a push | `WaypointFollower` applied the **forward-cone** stop even when it only had to **turn in place**, so a robot nose-to-wall could never turn away (BLOCKED → same replan → BLOCKED …, forever) | [kinematic] found by ablation D (seed 2 livelocked 250 s); [unit] `test_follower_can_turn_away_from_a_wall_it_faces` |
| Robot slow | (a) cruise fixed at 0.30 m/s; (b) the follower slowed by `dist_to_current_waypoint / SLOWDOWN_DIST` — with waypoints every 0.3 m it **never exceeded ~0.6·V_MAX on a straight line** | code reading; [kinematic] ablation B |
| "Arm / object handling freezes" | No teammate log was available, so these are the defects in the code that produce a hang or a dead controller: (1) a Python exception anywhere in the FSM (e.g. `None` TCP while encoders are invalid in `RAISE`/`TRANSIT_UP`) **killed the Webots controller** — robot frozen, no report; (2) gripper "closed" required < 0.2 mm finger motion **per step** — a force-limited finger jittering on a parcel never satisfied it, so CLOSE_GRIPPER only ended at its timeout; (3) steps without an own timeout waited a flat **60 s**; (4) under load the arm can stop a little short of the goal → SETTLE_TIMEOUT → recover/retry loop; (5) the first STOW command at start-up fails if the encoders are still NaN; (6) `arm.update()` was called twice per control step. | [unit] `tests/test_manipulation_robustness.py` reproduces each one |
| Recovery FSM timing wrong | `recovery_behaviors.RecoveryBehavior` used the **wall clock** (`time.time()`), unrelated to simulated time (fast mode = timeouts far too short) | code reading; fixed with an injectable sim clock |

---

## 2. What changed (by area)

### 2.1 Manipulation (Priority 1)
* `pick_place.py` — one arm/gripper update per control step; exceptions in a step's start/poll
  become `FAILED: EXCEPTION …` (never crash the controller); arm-step deadline = planned
  trajectory time + `ARM_MOTION_TIMEOUT_MARGIN` + `ARM_STEP_TIMEOUT_SLACK` (was 60 s);
  `WAIT_SENSORS` step before the start-up STOW (bounded by `ARM_SENSOR_WAIT_TIMEOUT`);
  recovery STOW falls back to a joint-space move when the straight line is infeasible; on
  timeout the arm is held.
* `gripper.py` — closing is finished when, over `GRIPPER_STALL_TIME`, no finger moved more
  than `GRIPPER_STALL_WINDOW_TOL` (window test), **or** both pads report contact for
  `GRIPPER_CONTACT_DONE_TIME` with a stable width. Grasp verification (width + palm + force/latch)
  is unchanged — nothing is declared held without sensors.
* `arm_controller.py` — after the trajectory ends, a residual ≤ `ARM_JOINT_TOL_LOADED` is
  accepted only once the arm has **stopped** (`ARM_SETTLE_STILL_TIME`); every fault logs a
  per-joint diagnosis (target, measured, error, torque feedback vs max torque, near-limit).
* `arm_driver.py` — motor torque feedback enabled (`read_torques()`, diagnosis only).
* `decision_maker.py` — in every `ARM_*` / `VERIFY_*` state the base command is forced to
  exactly (0, 0) (no deceleration ramp carried over from docking).
* `intellibot_controller.py` — `fsm.update()` is wrapped: an exception is printed with a
  traceback, the base stops, the arm holds, the mission goes to ERROR with the reason.

### 2.2 EKF integrated into the controller (Priority 2, Phase 11)
New `localization.py` (`LocalizationManager`) is **the pose the robot drives with** —
FSM, follower, planner, docking, package estimator, dashboard and heartbeat all use it.
* initialised from the first **valid** GPS + IMU sample;
* prediction from **signed** encoder increments (`Odometry.last_ds / last_dtheta`, wrapped);
* motion-dependent process noise `Q = Q_base + diag(a1·ds², a1·ds², a2·dθ² + a3·ds²)`;
* GPS / IMU health (staleness, invalid samples, gate-rejection rate) → trust → **R inflation**;
* **kidnapped-robot recovery**: GPS rejected for `EKF_KIDNAP_TIME` with mutually consistent
  fixes (spread ≤ `EKF_KIDNAP_MAX_SPREAD`) → re-initialise on GPS + IMU, report the jump;
  single outliers are still rejected;
* GPS dropout → dead reckoning, status `DEAD_RECKONING`, speed capped;
* ablation modes `ekf | odom | gps` (`config.LOCALIZATION_MODE`, env `INTELLIBOT_LOC_MODE`).
* Supervisor ground truth is **only** logged (`x_gt, y_gt, theta_gt` in run.csv) and used
  for RMSE / NEES in `mission_report.json` → `localization_vs_ground_truth`.

### 2.3 Dynamic replanning from the CURRENT pose (Priority 1)
* `dynamic_map.py` (`DynamicObstacleMap`) — continuous LiDAR fusion into the grid's
  existing dynamic layer (the layer A* already reads): hits gain evidence, cells a ray
  passes through lose it (observed free → cleared, no ghosts), unobserved cells keep it
  (memory of a blocked route) until `DYN_TTL`; hits on the known static map are ignored
  (static vs dynamic); two hits needed; own footprint and the target parcel never marked;
  skipped while localisation is unhealthy; records which cells were **vacated** → moving
  obstacles are recognised.
* `decision_maker.py` — `_nav_monitor()` every navigation step. Triggers: verified obstacle
  in the route corridor ahead (`REPLAN_LOOKAHEAD`), deviation (`REPLAN_DEVIATION_MAX`),
  stuck (`STUCK_TIME`), bumper collision, localisation pose jump, follower safety stop;
  cooldown `REPLAN_COOLDOWN` (pose jumps / collisions bypass it). Replan = A* from the
  **current EKF pose** to the **same final goal**, new `WaypointFollower` (index and heading
  PID reset). Moving obstacles are **yielded to** (they are not detoured around — that chases
  them); a route that crosses the recent lane of a moving obstacle that is close / approaching
  waits at the crossing (bounded by `DYN_LANE_MAX_WAIT`, then commits — no deadlock);
  never stops inside such a lane.
* `path_utils.plan_to(..., safe_start_radius)` + `find_safe_start()` — if the robot's cell is
  inside the inflated map, the nearest free cell that can be reached by a **straight move
  checked against the real obstacles** becomes the start (≤ `SAFE_START_RADIUS`). No
  teleporting, obstacles never cleared; if none exists → `START_BLOCKED` → recovery.
* Recovery (existing `recovery_behaviors.py`, now on simulation time):
  `WAIT/BRAKE → ROTATE only if the swept circle is clear → BACK UP only if the rear LiDAR
  sector is valid AND clear → REPLAN`, bounded by `NAV_MAX_RECOVERIES`, then ERROR with the
  reason. After a bumper contact the robot first moves `COLLISION_ESCAPE_DIST` away from the
  contact side (only if that side is verified clear). A safety stop repeated without progress
  escalates to recovery instead of replanning the same route forever.
* `waypoint_follower.py` — turning in place checks the body's swept circle, not the forward
  cone; speed ramps down on the **remaining route length**, not the distance to the next
  waypoint.

### 2.4 Safety layer and speed governor (Priority 1)
`safety.py`:
* `stopping_distance(v) = v²/(2·a_brake) + v·t_react + d_margin` with
  `SAFETY_BRAKE_DECEL = 0.8 m/s²`, `SAFETY_REACTION_TIME = 0.10 s`,
  `SAFETY_STOP_MARGIN = 0.10 m`; its inverse `clearance_speed(c)`.
* `corridor_clearance()` — free distance in the swept corridor (body half width + 5 cm) in the
  **direction of travel** (rear for reversing), minus the body/parcel extent.
* `SpeedGovernor.limit()` on every base command:
  `v_cap = min(V_CRUISE 0.45, v_clear, health cap, zone cap)`; zone caps: carrying 0.35 m/s,
  goal approach ramp; health caps: LiDAR NaN fraction / failure, localisation
  DEAD_RECKONING / GPS_REJECTED → 0.12 m/s, LiDAR failed → stop. One scale factor for (v, w)
  (curvature preserved), acceleration / deceleration limits, emergency stop at the margin
  and on bumper contact / follower safety stop (no ramp). Docking runs in DOCK mode
  (docker keeps control of the final approach; only caps and ramps apply).
* Mission report: `safety.min_clearance_m`, `emergency_stops`, `near_miss_steps`,
  `collisions_bumper` (rising-edge events, not one per step as before).

### 2.5 Cameras
Preserved (front + rear pipeline, health, detections). Camera health transitions are now in
`mission_report.json → camera_health` and on the dashboard. Existing offline camera tests
pass. **Rendering in Webots is not verified by this branch.**

### 2.6 Shelves
Unchanged (no genuine defect found). `RackOccupancy.summary()` added for the dashboard.

### 2.7 Phases 16 / 17 / 18
* **16 audit** — STORE flow unchanged and re-verified [kinematic]: 3/3 missions DONE with the
  full stack. RETRIEVE (S2) is **not implemented** (building blocks exist: `start_pick` at any
  reachable height, `start_place`; missing: slot → parcel lookup and a RETRIEVE goal in the FSM).
  `run_scenario.py` gained the Webots ablation scenarios `A_odom_only_P3_B`,
  `A_gps_only_P3_B`, `A_fixed_speed_P3_B`, `A_static_replan_P3_B`.
* **17 dashboard** — Display now shows the EKF pose, the route, dynamic obstacle cells (red),
  the ground-truth marker (evaluation overlay, green ring), localisation status / σ /
  relocalisations, GPS/IMU/LiDAR/camera health, replans / recoveries, speed cap and its
  reason, clearance, bumper counts, carrying flag, shelf occupancy; text turns red when
  localisation is not OK or an emergency stop is active. No web dashboard (feature freeze).
* **18** — `tools/run_experiments.py` runs the ablation matrix on the kinematic simulator and
  writes `docs/evidence/experiments/ablation_results.{md,json}`; Webots ablations via
  `run_scenario.py A_*`.

---

## 3. Replanning behaviour after a displacement (what the robot does now)

1. Robot is pushed / dragged; wheels do not see it.
2. GPS fixes now disagree with the prediction → rejected by the gate (log: `GPS fix rejected
   by the gate … holding the prediction`).
3. After `EKF_KIDNAP_TIME = 0.5 s` of consistent rejected fixes:
   `[LOC] RELOCALISED on GPS+IMU … jump (+0.30, +0.50) m, +35 deg`.
4. FSM: `Robot displaced by (…) — replanning from the CURRENT pose` →
   `REPLANNING from (x_now, y_now) to (goal) [trigger POSE_JUMP]` → new follower.
5. If the new pose is inside an inflation zone, the safe-start search moves the start out along
   a verified-free straight line (logged with the shift), else recovery.

[kinematic] ablation D: 3/3 DONE with relocalisation (pose RMSE 0.045–0.046 m) vs the old
behaviour (no relocalisation): 3/3 DONE but RMSE 0.13–0.18 m and up to +40 s mission time.
In the kinematic model the stale-pose robot eventually recovers because the gate re-accepts
GPS once P has grown; in Webots the reported symptom (driving back to the old route) is what
the stale pose produces — **Webots pending**.

---

## 4. Obstacle avoidance and speed control — offline numbers

All from `python tools/run_experiments.py` (kinematic, 107 s run, 2026-11-10):

* **Speed** (B): navigation time −15 % (26.6→22.6 s, 29.1→24.8 s, 28.5→24.5 s), mission time
  −2…−5 %, min clearance unchanged (0.20–0.22 m), 0 contacts, 0 emergency stops, parcel in
  slot 6/6. Most of a mission is docking + manipulation, which the governor does not speed up.
* **Static obstacle on the route** (C): legacy 96.4 s, stops at 0.30 m, waits 8 s, min clearance
  0.104 m; dynamic layer 85.3 s, detours 1.5 m ahead, min clearance 0.178 m, 0 contacts.
* **Crossing cart (0.4 m/s, 3 phases)** (C): legacy 3 contacts total, full 1 contact total;
  full is slower in phase 0 (104.7 vs 92.0 s, it waits at the crossing). One contact remains in
  phase 1.5 s for both — the kinematic cart drives through a stopped robot; Webots behaviour
  pending.
* **Localisation** (A, GPS σ 5 cm, IMU σ 1°, 2 % encoder scale error): EKF RMSE 0.009–0.012 m,
  3/3 DONE (+ DONE through a 10 s GPS dropout); GPS-only RMSE 0.070 m, 0/4 DONE (docking
  SETTLE never sees a still pose); odometry-only RMSE 0.22–0.26 m, 2/4 runs DONE (the dropout run equals seed 0: odometry ignores GPS). Mean NEES of the
  EKF 1.1–1.6 (< 3 → slightly conservative).

---

## 5. EKF wiring (data flow)

```
encoders ──► Odometry.update ──► (last_ds, last_dtheta) SIGNED ─┐
GPS ───────────────────────────────────────────────────────────►├─► LocalizationManager.step
IMU yaw ───────────────────────────────────────────────────────►┘      │ predict (motion Q)
                                                                       │ IMU update (R·1/trust)
                                                                       │ GPS update (R·1/trust, χ² gate,
                                                                       │            kidnap recovery)
                                                                       ▼
                       pose_est ──► DecisionMaker / follower / A* / Docker / estimator / dashboard
                       status   ──► fsm.health → SpeedGovernor caps, DynamicObstacleMap trust
                       pose_jump──► fsm.notify_pose_jump → replan from current pose
Supervisor ground truth ──► run.csv x_gt…, report RMSE/NEES   (EVALUATION ONLY)
```

---

## 6. Arm / gripper changes (summary)
See 2.1. Behavioural guarantees, each covered by a test in
`tests/test_manipulation_robustness.py`: close finishes despite jitter; missed grasp still
detected; small loaded residual accepted only when stopped; large residual still faults with a
per-joint diagnosis; tracking fault names the blocked joint; step exceptions fail cleanly;
NaN TCP in recovery fails cleanly; arm step deadline follows the motion (< 20 s instead of
60 s); start-up STOW waits for encoders (bounded); one arm update per control step.

---

## 7. Files

New: `controllers/intellibot_controller/localization.py`, `safety.py`, `dynamic_map.py`;
`tools/run_experiments.py`; `tests/test_manipulation_robustness.py`, `test_localization.py`,
`test_safety_navigation.py`, `test_integration_navigation.py`;
`docs/evidence/experiments/ablation_results.{md,json}`; this report.

Modified: `arm_controller.py`, `arm_driver.py`, `gripper.py`, `pick_place.py`,
`decision_maker.py`, `waypoint_follower.py`, `path_utils.py`, `recovery_behaviors.py`,
`ekf_localization.py` (verbose flag only), `kinematics.py` (signed increments),
`intellibot_controller.py`, `display_dash.py`, `logger.py` (new run.csv columns appended),
`shelving.py` (summary), `config.py` (new constants, one block per feature);
`tools/offline_mission_sim.py` (navigation / localisation / obstacle / displacement options),
`tools/run_scenario.py` (ablation scenarios); `tests/test_controller_smoke.py`; docs.

---

## 8. Test results (this machine, offline)

* `python -m pytest tests/ -q` → **303 passed** (244 before this branch; no test deleted, no
  assertion weakened).
* `python -m compileall controllers tools tests` → OK.
* `python tools/validate_world.py` → all worlds OK; `tests/test_world_generation.py` passes.

---

## 9. Known limitations (honest)
* Nothing here has run in Webots. Contact dynamics, wheel slip, real LiDAR noise and the
  Connector physics may change the numbers.
* Moving-obstacle handling has no velocity prediction beyond "vacated cells + approaching";
  one contact remains in one crossing-cart phase (kinematic).
* The governor's braking deceleration (0.8 m/s²) is an assumption — measure it in Webots
  (command 0.45 → 0 m/s, log the distance) and set `SAFETY_BRAKE_DECEL`.
* Without teammate logs the manipulation freeze root cause is inferred from the code; the
  diagnosis logging added here will show the exact joint/torque if it happens again.
* RETRIEVE (S2) not implemented.

---

## 10. Teammate handoff — Webots verification (GPU laptop)

```bash
git fetch origin
git switch fix/ankit-ekf-navigation-manipulation
python -m pytest tests/ -q                     # expect 303 passed
python tools/validate_world.py                 # worlds parse, devices/DEFs present
# Webots R2025a: open worlds/warehouse.wbt, press Play. Then batch runs:
python tools/run_scenario.py S1_store_P3_B --runs 5
python tools/run_scenario.py A_fixed_speed_P3_B --runs 3
python tools/run_scenario.py A_static_replan_P3_B --runs 3
python tools/run_scenario.py A_odom_only_P3_B --runs 3
python tools/run_scenario.py pick_check
```

Check in Webots and report back (logs are in `logs/final/<scenario>/run_XX/`):
1. Console shows `[LOC] … initialised (ekf)` and `navigation: localization=ekf, dynamic
   replanning=ON, speed governor=ON`.
2. **Displacement test**: during DELIVER, pause, drag the robot ~0.5 m sideways, resume →
   expect `RELOCALISED` then `REPLANNING from (<new pose>) … [trigger POSE_JUMP]`, and the
   robot does **not** drive back to the old spot.
3. **Obstacle test**: drop a box on the delivery route → expect `OBSTACLE DETECTED on the
   route … m ahead` and a detour without stopping at 0.30 m; red cells on the Display.
4. Speed: `run.csv` `v_cmd` reaches ~0.45 m/s on straights, `v_cap`/`cap_reason` columns
   make sense; `mission_report.json → safety.min_clearance_m`, `collisions_bumper`.
5. Arm: full pick + place; if anything stalls, copy the `[ARM]` / `[GRIPPER]` / `[MANIP]`
   lines (they now include the per-joint diagnosis) and `manip.csv`.
6. Measure braking: log distance from `v_cmd` 0.45 → 0; if > 0.25 m, lower
   `SAFETY_BRAKE_DECEL` in `config.py`.
