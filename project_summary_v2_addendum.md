# PROJECT_SUMMARY_V2_ADDENDUM.md — Full-scope technical specification (Mid + Final)

> **Precedence:** this addendum **extends and, where they conflict, overrides** `project_summary.md` (§2 scope table, §7 world, §8 robot, §11 sample reuse, §14 ownership, §15 hooks). Everything in `project_summary.md` about conventions (units, frames, ENU, Webots-free core, config-driven constants, "verify don't assume", "don't fake results") **still applies**.
> Work order is in `checklist_v2.md`.

---

## A0. What changed (v1 → v2)

| Area | v1 (mid-eval, single robot) | **v2 (final)** |
|---|---|---|
| Robots | 1 differential-drive robot | **2–3 robots working simultaneously** + a Fleet Manager |
| Manipulation | placeholder pick, no arm | **Mobile manipulator**: vertical lift + planar 2R arm + sensorized parallel gripper |
| Objects | 3 packages on the floor | **Pile of boxes** (stacked, multiple IDs) at a receiving area; **shelves with slots** (levels × bays); inventory |
| Task types | "pick P3 → Zone B" | **STORE** (pile → assigned shelf slot) and **RETRIEVE** (shelf slot → dispatch zone) |
| Sensing | LiDAR, camera, GPS, IMU, encoders | + **rear camera, IR ring, ultrasonic, light sensor, bumpers/tactile, gripper force + palm IR**, noisy/faulty sensor models |
| Estimation | GPS+IMU pose (idealized) | **Multi-sensor Kalman filtering (EKF)** + sensor health monitor + multi-sensor obstacle verification; works when lights go off or a sensor fails |
| Navigation | A* + PID follower + LiDAR stop | **Layered costmap, live re-planning, moving-obstacle tracking & prediction (Kalman), DWA local planner, rear-safety, recovery behaviors** |
| Obstacles | static | static + **moving obstacles** (carts/workers) + **other robots** |
| Coordination | n/a | task allocation, traffic management (priorities/reservations/one-way aisles), comms with dropout |

**Honest feasibility note for the agent:** this is a lot. Build in the tier order given in `checklist_v2.md` (Tier 1 → 2 → 3). A working Tier 1 beats a broken Tier 3. Every feature needs a **fallback** (see §A13).

---

## A1. Interpretation notes (sensors you will see in requirements)

- **"UV" sensor:** Webots has **no native UV sensor**. We interpret the requested sensor as **ultrasonic (sonar)** — implemented with a `DistanceSensor` of `type "sonar"`. If a UV-*light* modality is wanted, emulate it as "UV-reflective marker tags" detected with a `LightSensor`/camera channel and say clearly it is *emulated*. Do not claim a real UV sensor.
- **IR** → `DistanceSensor` `type "infra-red"` (short range, narrow beam).
- **Tactile** → `TouchSensor` `type "bumper"` (chassis) and `type "force"` / `"force-3d"` (gripper fingers).
- Webots sensors are ideal by default. **Add noise** (node `noise`/`accuracy`/`resolution` fields where they exist, or software noise in `sensor_models.py`) so that fusion has something to fix. Always keep a clean ground truth from the Supervisor for evaluation only (never feed ground truth to the robot's decision logic).

---

## A2. Sensor suite & coverage

| Sensor | Webots node / type | Placement | Measures | Typical failure it must survive | Redundancy partner(s) |
|---|---|---|---|---|---|
| 2D LiDAR | `Lidar` 360 rays, 0.05–6 m | center-top | range scan | dropout/frozen scan (injected) | IR ring, sonar, camera |
| Front camera | `Camera` ~640×480 | front | color/ArUco, bearing | **lights off**, glare | LiDAR, IR, sonar |
| **Rear camera** | `Camera` ~320×240 | rear, facing back | approaching robot/obstacle, rear docking | lights off | rear IR/sonar, LiDAR |
| IR ring (6–8) | `DistanceSensor` infra-red, 0.1–0.8 m | FL, FR, L, R, RL, RR (+front/rear) | near-field range | noise, specular | sonar, LiDAR |
| Ultrasonic ×2–4 | `DistanceSensor` sonar, 0.2–3 m | front, rear (+sides) | wide-cone range | poor angular resolution, multipath | LiDAR, IR |
| Light sensor | `LightSensor` | top | ambient light level | — | used to **detect lights-off** → trust model for cameras |
| Bumpers | `TouchSensor` bumper | front & rear skirts | contact (collision counter) | — | last-resort safety |
| Wheel encoders | `PositionSensor` | wheels | odometry | slip drift | IMU, landmarks |
| IMU | `InertialUnit` (+ optional `Gyro`) | body | yaw | noise/bias | encoders, landmarks |
| GPS-like beacon | `GPS` with `accuracy` | top | noisy x,y (low rate) | dropout (injected) | landmarks, odometry |
| Landmark detection | from LiDAR clusters / camera | warehouse posts at known (x,y) | range-bearing | camera blind in dark | LiDAR landmarks |
| Joint sensors | `PositionSensor` on lift/arm/gripper | arm | joint states | — | — |
| Gripper force | `TouchSensor` force on each finger | inner finger faces | grasp force, slip | — | palm IR (width/presence) |
| Palm IR | `DistanceSensor` infra-red | gripper palm | object distance/presence, slot-occupancy check | — | hand camera |
| Hand camera (optional) | small `Camera` | wrist | fine alignment | dark | palm IR |

**360° coverage rule:** front, rear, left, right must each be covered by **≥2 independent modalities** (e.g., rear = LiDAR + rear sonar + rear IR + rear camera + rear bumper). Create a coverage table in `docs/sensor_coverage.md` and a test that asserts it from `config.py`.

---

## A3. Noise and fault model (`sensor_models.py`, `fault_injector.py`)

Noise (Gaussian, config-tunable σ): wheel slip scaling on encoders, IMU yaw σ≈0.5–1°, GPS σ≈0.05–0.15 m at 2–5 Hz, LiDAR σ≈1–2 cm, IR σ≈1 cm (+ range-dependent), sonar σ≈2–3 cm with occasional outliers.

Faults (scheduled in config or triggered by the Fleet Manager/Supervisor at run time):

| Fault | How injected | Expected system behavior |
|---|---|---|
| **Lights off** | Supervisor sets the `on` field of warehouse lights to FALSE (or intensity ↓) | light sensor drops, camera health → FAILED/DEGRADED; fusion de-weights camera; navigation continues on LiDAR+IR+sonar; vision-dependent tasks (ArUco read) pause or use fallback (known slot coordinates + palm IR) |
| LiDAR dropout / frozen | software: return inf/stale scan for T s | health monitor flags; IR/sonar take over near-field; robot slows (reduced speed cap) |
| GPS dropout | stop providing GPS for T s | EKF relies on odometry+IMU+landmarks; covariance grows (visible in log) |
| Camera blind / glare | brightness saturate / zero | same as lights off |
| Stuck-at / bias drift | software | innovation gating rejects; health monitor trust ↓ |
| Comm loss (multi-robot) | Receiver ignores messages for T s | robots revert to local sensing, conservative speed, no new reservations |

---

## A4. Kalman filtering & sensor fusion (core new feature)

### A4.1 Pose EKF (`ekf_localization.py`, pure Python)
State **x = [x, y, θ]ᵀ**, covariance **P**.

**Predict** (each control step, from encoder increments Δs, Δθ):
```
x' = x + Δs·cos(θ + Δθ/2)
y' = y + Δs·sin(θ + Δθ/2)
θ' = θ + Δθ
F = [[1, 0, -Δs·sin(θ+Δθ/2)],
     [0, 1,  Δs·cos(θ+Δθ/2)],
     [0, 0,  1]]
P' = F P Fᵀ + Q          # Q from odometry noise (σs ∝ |Δs|, σθ ∝ |Δθ| + slip term)
```
**Update** (generic): `y = z − h(x)`, `S = H P Hᵀ + R`, `K = P Hᵀ S⁻¹`, `x ← x + K y`, `P ← (I − K H) P` (use Joseph form for stability). **Wrap angle components of y to [−π, π].**

| Measurement | z | h(x) | H | R |
|---|---|---|---|---|
| IMU yaw | θ_imu | θ | [0 0 1] | σψ² |
| GPS/beacon | (x, y) | (x, y) | [[1 0 0],[0 1 0]] | σg² I (inflate if degraded) |
| Landmark i at known (lx, ly) (LiDAR cluster or camera) | (range, bearing) | (√((lx−x)²+(ly−y)²), atan2(ly−y, lx−x) − θ) | Jacobian of h | diag(σr², σφ²) — **camera R × large factor when light is low** |

**Gating:** accept a measurement only if Mahalanobis d² = yᵀS⁻¹y < χ²(dof, 0.99) (dof 1: 6.63, 2: 9.21, 3: 11.34). Rejected measurements are counted per sensor (feeds health monitor).

**Consistency check (for the report):** NEES ε = (x_true − x̂)ᵀ P⁻¹ (x_true − x̂), averaged over runs, should lie within χ² bounds; also report RMSE for position and heading **with vs. without fusion** in every scenario (nominal, lights-off, GPS dropout, LiDAR dropout).

### A4.2 Sensor health monitor (`sensor_health.py`)
Per sensor, every step compute: `age` (staleness), validity ratio (NaN/inf), plausibility (range limits, max rate of change), a **modality-specific quality** (camera: mean brightness + contrast/entropy; LiDAR: fraction of valid rays, scan change; encoders: wheel-speed consistency), and the running **NIS** (normalized innovation squared) from the EKF. Output per sensor `health ∈ {OK, DEGRADED, FAILED}` and `trust ∈ [0,1]`. Policy: `R_eff = R / max(trust, ε)`; FAILED ⇒ measurement not used. Log every health transition (`[t=41.2s] camera: OK -> FAILED (brightness 3.1)`).

### A4.3 Multi-sensor obstacle fusion (`obstacle_fusion.py`)
Divide surroundings into sectors (e.g., 36 × 10°). For each sector combine readings from LiDAR, IR, sonar, (depth camera if present), and tracked-object/comm data:
```
r_fused = Σ (r_i / σ_i²) / Σ (1 / σ_i²)          σ_fused² = 1 / Σ (1 / σ_i²)
```
with σ_i scaled by health/trust. **Verification:** an obstacle is `VERIFIED` if ≥2 independent modalities agree within tolerance; `UNVERIFIED` if only one reports it (still honored conservatively by the safety layer, but flagged and, if persistent, down-weighted to detect a faulty sensor); `CONFLICT` when sensors disagree strongly → reduce speed, rely on the most trusted, raise a log event. Output `ObstacleReport(sector, range, confidence, sources)`. Safety stop uses the conservative (min) over *trusted* sources; planning uses fused values.

**Rule for the report:** the system "picks the best sensor" implicitly via inverse-variance weights + gating + health, explicitly via the verification logic. Do not describe it as a hard switch unless you implement one.

### A4.4 Fusion also for manipulation
Grasp/placement decisions use fusion of: gripper force + finger position (width), palm IR distance, camera/ArUco pose (when lit), joint encoders. Example: "box present" = width within ±1 cm of expected AND force ≥ threshold; "slot free" = palm IR reading ≈ back-wall distance AND (if lit) camera sees empty slot.

---

## A5. Moving-obstacle tracking & prediction (`tracker.py`, pure Python)

1. **Segment** LiDAR points (world frame) into clusters by Euclidean gap threshold; discard clusters matching the known static map (inflated tolerance) and clusters that are **other robots** (known from broadcast poses).
2. **Associate** clusters to tracks by nearest neighbor with Mahalanobis gating; create/delete tracks with confirmation counters.
3. **Track** each with a constant-velocity Kalman filter: state `[x, y, vx, vy]`, `F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]`, `H = [[1,0,0,0],[0,1,0,0]]`.
4. **Classify** `STATIC` / `MOVING` (speed > ~0.1 m/s over N frames).
5. **Predict** positions over a horizon (2–3 s), write the swept area into the costmap's *predicted-dynamic* layer, and compute **time-to-collision** `TTC = d / closing_speed`.

---

## A6. Navigation stack v2

**Layered costmap** (`costmap.py`): static layer (known map) + inflation layer (robot radius + margin, with a decaying cost gradient) + dynamic layer (live LiDAR hits, cleared when seen free) + predicted-dynamic layer (tracker) + **reservation layer** (cells reserved by other robots at the time window). Combine to a single planning grid.

**Global planner:** A* (from v1) on the combined costmap; **re-plan triggers**: (a) `path_is_blocked` by dynamic/predicted layer, (b) costmap changed within the path corridor, (c) local planner stuck > T seconds, (d) reservation conflict. Log every re-plan (reason, old/new length, time). Optional: D* Lite (P2).

**Local planner — Dynamic Window Approach (`dwa.py`)**:
```
Dynamic window: v ∈ [v_c − a_max·dt, v_c + a_max·dt],  ω ∈ [ω_c − α_max·dt, ω_c + α_max·dt]
Admissible:     v ≤ sqrt(2 · d_obs(v,ω) · a_dec)      (can stop before collision)
For each (v,ω): simulate ~1.5–2.0 s with the unicycle model, check against costmap (incl. predicted-dynamic)
Score:          G = α·heading_to_goal_point + β·clearance + γ·speed + δ·path_adherence      (normalized)
Pick argmax G; if none admissible → recovery
```
Allow small **reverse** velocities **only if the rear is verified clear** (rear sensors).

**Rear safety (`rear_safety.py`)**: monitors rear/side-rear sectors and the rear camera. Rules: (1) never reverse unless rear sectors verified clear for ≥ X cm and no approaching track; (2) if an object/robot approaches from behind with `TTC < T_warn`: warn (LED/log/broadcast), `TTC < T_yield`: move aside/forward along a free direction or stop and yield per traffic rules; (3) when stopped, keep monitoring all sides. Test it explicitly (scenario S6).

**Recovery behaviors:** wait (bounded) → rotate in place (if sides clear) → back up (if rear clear) → re-plan with inflated costs → request help / report ERROR.

**Speed governor:** `v_cap = min(V_MAX, k·d_nearest, f(health), f(zone))` — slower near obstacles/people/shelves, with degraded sensors, and inside docking zones.

---

## A7. World v2 (`worlds/warehouse.wbt`, generated)

Suggested **12 m × 8 m** arena (x∈[−6,6], y∈[−4,4]), grid 0.1 m (120×80). All values in `config.py`; the generator writes the world; the grid is built from the same config.

| Zone / object | Spec |
|---|---|
| Receiving area ("pile") | floor pallet region ~1.2 × 1.0 m with a **pile of 9–12 boxes**: 2 layers (e.g., 3×2 footprint × 2 layers), box size 0.20–0.25 m, mass ~0.3–0.5 kg, each with an **ArUco ID marker** on its front/top face + a category color band. IDs P1…P12 |
| Shelves | 4–6 shelf units, each **3 levels × 3 bays = 9 slots**; slot IDs `SHk-Lj-Bi`; shelf surface heights ≈ 0.15 / 0.60 / 1.05 m; slot interior ≥ 0.35 (w) × 0.40 (d) × 0.40 (h); each slot has an ArUco tag at its front edge; back wall at known distance (for IR occupancy check) |
| Dispatch Zones A and B | floor patches (yellow / cyan) with a drop spot each |
| Aisles | ≥ 1.6 m wide (two robots must be able to pass) **or** marked one-way; ≥ 2 distinct routes between receiving and any shelf |
| Landmarks | 6–8 slim posts at known positions (for EKF landmark updates) |
| Robot docks (parking) | 3 bays for R1–R3 start/idle poses |
| Lights | ≥ 3 lights with `DEF` names (`LIGHT_1…`) so a *zone* or the *whole* warehouse can go dark via Supervisor |
| Moving-obstacle lanes | 2–3 lanes crossing aisles (see §A8) |
| Walls | 4 walls, plain colors; floor neutral gray |

Keep package/zone/shelf colors in distinct HSV ranges. Textures for ArUco markers: generate PNGs with `cv2.aruco` (dictionary `DICT_4X4_50` or similar) into `worlds/textures/`; apply with `ImageTexture`. Verify the marker is readable at 0.5–1.5 m in the rendered camera image before relying on it.

**Inventory model** (`inventory.py`, pure): `slots: dict[slot_id → pkg_id|None]`, `packages: dict[pkg_id → {category, dest_zone, size, mass}]`, `assign_slot(pkg)` policy (heavy → lower levels; same category grouped; nearest free to receiving; reserve a slot while a robot is carrying), `release/confirm`. Owned by the Fleet Manager; robots also verify physically.

---

## A8. Moving obstacles (supervisor-driven, physics-enabled)

- `DEF CART_1`, `CART_2`, optional `WORKER_1` (a person-sized shape or the Webots pedestrian asset if the installed version has it): **Solids with physics**, moved by the Fleet-Manager/Supervisor using `Node.setVelocity([vx,vy,0,0,0,0])` along waypoint lanes, so they collide realistically and robots can't phase through them. Behaviors: patrol (loop), cross-aisle at scheduled times, **stop-and-go**, random pop-up (a box dropped into an aisle).
- Obstacle motion parameters (speed 0.2–0.6 m/s) in `config.py`; seeds for reproducibility.
- Robots must **not** receive these positions from the supervisor; they must sense them (LiDAR/IR/sonar/camera) and track them with the Kalman tracker.

---

## A9. Mobile manipulator (Robot v2)

### A9.1 Chassis (parametrized in `config.py`)
Body ≈ 0.55 × 0.40 × 0.18 m; wheel radius 0.08 m; wheel separation ≈ 0.42 m; mass low and centered (battery plate low), 2 passive casters (frictionless contact), enough wheelbase to avoid tipping when the arm is extended and loaded. Planning radius ≈ 0.34 m circumscribed + margin 0.06 → **inflate ≈ 0.40 m**. Arm stowed **inside** the footprint when driving.

### A9.2 Arm (recommended: lift + planar 2R + wrist yaw + gripper)
- **Lift:** `LinearMotor` along z, 0–1.1 m (speed ≤ 0.1 m/s), carries the arm base. Sets height.
- **Planar 2R arm** (revolute joints about z, horizontal plane at lift height): `l1 ≈ 0.30 m`, `l2 ≈ 0.28 m` (reach ≈ 0.58 m), joint limits ±~120°.
- **Wrist yaw** keeps the gripper pointing forward (parallel to shelf depth).
- **Parallel gripper:** two finger slides (`LinearMotor`), opening 0–0.30 m, finger length ≈ 0.10 m, high-friction contact material, `maxForce` limits.
- The base handles gross positioning/heading; the lift handles height; the 2R arm handles reach + small lateral error (this **relaxes docking tolerance to ~±3 cm lateral**).
- **Fallback tier A (Cartesian):** lift + linear extension + gripper (no revolute joints) if 2R physics is troublesome. **Fallback tier B:** `VacuumGripper` or `Connector`-based pick (document as a simplification).

### A9.3 Kinematics (`arm_kinematics.py`, pure + tests)
Planar 2R IK for target (x, y) in the arm-base frame:
```
c2 = (x² + y² − l1² − l2²) / (2·l1·l2)           # reject if |c2| > 1 (out of reach)
θ2 = ± acos(c2)                                   # elbow-left / elbow-right (choose by joint limits/collision)
θ1 = atan2(y, x) − atan2(l2·sinθ2, l1 + l2·cosθ2)
θ3 = φ_desired − θ1 − θ2                          # wrist yaw so the gripper faces forward
FK: x = l1·cosθ1 + l2·cos(θ1+θ2);  y = l1·sinθ1 + l2·sin(θ1+θ2)
```
Homogeneous transform chain for perception→grasp (spatial transformations):
`T_world_box = T_world_robot · T_robot_lift/arm_base · T_robot_camera · T_camera_marker`, where `T_camera_marker` comes from `cv2.aruco.estimatePoseSingleMarkers`/`solvePnP` using intrinsics derived from the Webots camera FOV: `fx = fy = (W/2)/tan(FOV/2)`, `cx = W/2`, `cy = H/2`.

**Trajectory generation (`trajectory.py`):** joint-space quintic profile `s(τ) = 10τ³ − 15τ⁴ + 6τ⁵`, `q(t) = q0 + (q1 − q0)·s(t/T)` (zero velocity/acceleration at ends), with T from joint speed limits; Cartesian waypoints (pre-grasp, grasp, lift, retract) mapped through IK. Unit-test IK↔FK round trip and workspace coverage of every shelf slot and every pile position from the planned docking poses.

### A9.4 Pick sequence (state machine, each step has a sensor check)
`PRE_DOCK → ALIGN (marker/LiDAR) → OPEN_GRIPPER → APPROACH (pre-grasp) → ADVANCE (palm IR distance) → CLOSE (until force ≥ F_grasp on both fingers) → VERIFY (width ≈ box width ±1 cm, force stable) → LIFT 5 cm (check force/slip) → RETRACT → STOW → CARRY`. Failure branches: not found → re-scan; missed (width too small) → retry (max 2); slip → regrasp; arm blocked → abort + report.

### A9.5 Place sequence (onto a shelf slot)
`PLAN_DELIVERY → DOCK at bay (marker + LiDAR line-fit + sonar/IR distance) → LIFT to level height + clearance → CHECK SLOT FREE (palm IR vs expected back-wall distance; camera if lit) → EXTEND (IK) → LOWER to surface (+1–2 cm) → OPEN → VERIFY RELEASE (force ≈ 0) → RETRACT → STOW → CONFIRM (camera sees ID in slot or IR shows occupied) → update inventory`. Failure branches: slot occupied → request another slot; verification fails → retry/alert.

### A9.6 Physics hints (Webots)
Contact friction between fingers and boxes ≥ 1.0 via `ContactProperties`; fingers have simple box `boundingObject`s; use position control with velocity limits and `maxForce`; keep arm mass small relative to base; use `basicTimeStep` 8–16 ms; box mass ≤ 0.5 kg; if boxes jitter in the grip, raise damping/friction, slow the closing, and use force-limited closing. Record settings that worked in `docs/webots_findings.md`.

---

## A10. Multi-robot system (2–3 robots + Fleet Manager)

**Processes:** N robot controllers (same code, `controllerArgs ["R1"]`, `["R2"]`, `["R3"]`; read via `sys.argv`/`robot.getName()`), and one **Fleet Manager** controller (Supervisor-enabled `Robot`) that: owns the task queue + inventory, allocates tasks, runs the fault injector, drives moving obstacles, logs ground truth, and renders the dashboard.

**Communication:** `Emitter`/`Receiver` pairs (distinct channels per robot + a broadcast channel; limited `range`; optional message drop). Messages (JSON):
- `HEARTBEAT {id, t, pose(x,y,θ,P), v, ω, state, task_id, path_ahead[], health}` @ ~5 Hz
- `TASK_ASSIGN {task_id, type: STORE|RETRIEVE, pkg_id, src, dst_slot|zone}` / `TASK_ACCEPT` / `TASK_DONE` / `TASK_FAIL`
- `RESERVE_REQ {id, cells/segments, t_start, t_end}` / `RESERVE_GRANT|DENY`
- `YIELD_REQ`, `SLOT_ASSIGN_REQ`, `SLOT_ASSIGNED`, `INVENTORY_UPDATE`
Heartbeats double as a **fusion input**: another robot's broadcast pose is a sensor reading about that obstacle; compare against LiDAR tracks to verify (and to keep other robots out of the "unknown moving obstacle" class). Each robot carries a unique **color/marker on back and sides** so rear/side cameras can identify it.

**Task allocation:** greedy nearest-feasible (cost = A* path length + current queue + battery/none) → optional auction/Hungarian (P2). Constraint: a package/slot is reserved by at most one robot.

**Traffic management (priority order of implementation):**
1. One-way aisles + fixed right-of-way rule (P0)
2. Priority by robot ID/task urgency + yield/wait at intersections (P0)
3. Reservation table on corridor segments/cells with time windows; space-time A* with a short horizon (P1)
4. Deadlock detection (mutual waiting > T) + resolution (lower priority backs up if rear clear / takes alternate route) (P1)
5. Comm-loss mode: conservative speed, local-only avoidance (P1)

---

## A11. Mission FSMs

**Robot FSM (per robot):** `IDLE → GET_TASK → (STORE: GO_TO_PILE → SELECT_BOX → DOCK_PILE → PICK → PLAN_TO_SLOT → GO_TO_SHELF → DOCK_SHELF → PLACE → CONFIRM) | (RETRIEVE: GO_TO_SHELF → DOCK_SHELF → PICK_FROM_SLOT → GO_TO_ZONE → DROP) → REPORT → IDLE`, with orthogonal sub-states `NAV: FOLLOW | AVOID | REPLAN | RECOVER | YIELD`, global overrides `SAFE_STOP` (bumper/TTC) and `DEGRADED` (sensor faults), and `ERROR`. Every transition logged with timestamp and reason.

**Box selection from the pile:** pick an *accessible* box (top layer, free front face, ArUco ID readable) matching the task; if the required box is buried, (P2) move blockers to a buffer area first.

---

## A12. Metrics & experiments (all logged, N ≥ 10 where stochastic)

| Group | Metrics |
|---|---|
| Fusion/EKF | pos/heading RMSE and NEES: {no fusion (GPS+IMU or odometry only), EKF} × {nominal, lights-off, GPS dropout, LiDAR dropout}; sensor-health event log; false-reject rate of gating |
| Obstacle fusion | detection rate/latency per modality vs fused; verified/unverified/conflict counts; behavior under each fault |
| Navigation | success rate with moving obstacles, collisions (bumper count), minimum clearance, #re-plans + reasons, re-plan time, path length vs optimal, mission time, rear-approach avoidance success (S6) |
| Manipulation | pick success rate, place success rate, placement error (m) in slot, grasp retries, failure-mode breakdown, cycle time per pick+place |
| Multi-robot | throughput (packages/min) for 1, 2, 3 robots (speed-up), conflicts, yields, deadlocks, total wait time, comm-loss behavior |
| System | end-to-end success over repeated runs, CPU/real-time factor, logs reproducible with a seed |

Always compare against the **v1 baseline** (no fusion / no tracking / single robot) to show the value of each added feature (ablation table).

---

## A13. Risk register & fallbacks

| Risk | Fallback |
|---|---|
| Arm physics unstable / grasp unreliable | Cartesian (lift+extend) arm → vacuum gripper → Connector attach; document as simplification, keep sensor-verification logic |
| ArUco unreadable in Webots render | color bands + known slot coordinates + palm-IR verification; larger markers |
| Robot tips over with arm | wider/lower chassis, heavier base, limit speeds when arm extended, stow arm while driving |
| 3 robots too slow/unstable | run with 2 robots; reduce camera resolution; larger time step for non-manip phases; headless data runs for metrics |
| Real-time performance (Python) | keep grid ≤ 0.1 m, plan at ≤ 2 Hz, tracker/DWA at control rate with small samples, cache inflated maps, profile |
| Deadlocks in aisles | one-way aisles first; reservation timeouts; random back-off |
| Kalman tuning difficulty | start with GPS+IMU+odometry, add landmarks later; tune Q/R with ground-truth NEES; keep tuning log |
| Time overrun | follow Tier order in `checklist_v2.md`; freeze features at the "feature freeze" gate |

---

## A14. Webots sample assets to reuse (final-eval additions; **verify each by opening it first**)

| Sample folder | Use |
|---|---|
| `distance_sensor` | IR and sonar rings: lookup tables, types, noise |
| `light_sensor` | ambient-light sensing for lights-off detection |
| `bumper`/touch sensor sample (if present), `force_sensor`, `force3d_sensor` | bumpers; gripper finger force sensing |
| `linear_motor`, `motor*` | lift + gripper fingers + arm joints (position control, `maxForce`) |
| `range_finder` | optional depth for pile/slot perception |
| `emitter_receiver` / `EmitterReceiver` | multi-robot comms, channels, range |
| `sample_supervisor`, `gps_supervisor`, `display_supervisor` | Supervisor: moving obstacles (`setVelocity`/`setSFVec3f`), light toggling, ground truth, spawning |
| `display` | in-sim dashboard (map, tracks, inventory, task queue) |
| `connector`, `vacuum_gripper` | fallback pick mechanisms |
| `camera_recognition` | ground truth to validate vision (`Recognition` node) |
| `pen` | draw trails per robot |
| `led`, `speaker` | state indicators; audible rear-approach/reversing warning |
| `radar` | optional extra moving-object sensor (only if time) |
| Webots robot PROTOs (`projects/robots/`) | arm/gripper references (e.g., KUKA youBot mobile manipulator, Universal Robots arms, Robotiq grippers) for joint/gripper modeling ideas — **check availability in the installed version**; use as reference or fallback, not as a substitute for the required differential-drive + custom arm unless the fallback is invoked |

---

## A15. Ownership for the final phase (balanced for 3 people)

| Person | Final-eval scope |
|---|---|
| **A — Robot, Manipulation & Control** | Robot v2 chassis, lift/arm/gripper models, arm FK/IK, trajectory generation, grasp/place state machines, docking control, gripper force/palm-IR logic, DWA/follower integration with the new chassis, bumpers, actuator-side tests |
| **B — Perception, Sensor Fusion & Tracking** | sensor suite + noise/fault models, health monitor, **EKF**, obstacle fusion/verification, moving-obstacle tracker, rear sensing/rear-safety perception, ArUco/vision, lights-off/fault experiments, fusion metrics |
| **C — Planning, Fleet & Integration** | layered costmap, A* re-planning, DWA integration, FSMs, **Fleet Manager**, task allocation, traffic/reservations, comms protocol, inventory/slot assignment, moving-obstacle scenarios, dashboard, mission report, final integration + scenario runs |

Everyone: write their module's "how to explain it" paragraph, Q&A prep, and present their section.
