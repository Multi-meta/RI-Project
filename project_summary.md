# PROJECT_SUMMARY.md — IntelliBot / IntelliWarehouse

> **Audience:** the AI coding agent that will build this project.
> **Read this file fully first, then execute `checklist.md` top to bottom.**
> This file is the single source of truth for *what* we build, *why*, and *the rules/conventions*. `checklist.md` is the *order of work*.

---

## 1. What we are building

A **software-only simulation** (Webots + Python) of a miniature warehouse robot that performs an autonomous pick-and-deliver mission:

> Task: *"Pick package P3 and deliver it to Zone B."*
> Pipeline: **find package (camera) → plan route (A\*) → navigate (PID waypoint following) → avoid obstacles (LiDAR) → pick → re-plan → deliver → report.**

The point of the project is **robotic intelligence**, not a pre-recorded animation: the robot must *perceive, decide, plan, and control in a closed loop*. A hard-coded route is explicitly NOT acceptable.

Syllabus topics we want to visibly cover: spatial descriptions & transformations (robot↔world frames), differential-drive kinematics, locomotion, sensors (LiDAR, camera, GPS/IMU, encoders), robotics vision / image processing / object detection, navigation (A\*), autonomous control (PID / waypoint following), trajectory generation (path smoothing), manipulation (simulated pickup), industrial robotics, dynamic obstacle avoidance (final eval).

Team: 3 people. **Right now a single AI agent builds everything**; the work is later divided among the 3 humans, so code must be **modular, readable, commented, and explainable** (evaluators will question each member on how it works). See §14 for the module ownership map.

---

## 2. Two evaluations — scope split

| Area | **MID evaluation (build now)** | **FINAL evaluation (later; leave hooks)** |
|---|---|---|
| World | Full warehouse world: shelves, packages P1–P3, pickup area, Zone A/B, static obstacles | + dynamic obstacle spawned at runtime |
| Robot | Custom differential-drive robot with LiDAR, camera, GPS, IMU, encoders | + simulated pickup mechanism, bumper/collision counter |
| Kinematics | Forward/inverse kinematics + odometry, verified vs ground truth | — |
| Perception | LiDAR obstacle detection; OpenCV package detection; package world-position estimate | robustness (lighting), obstacle classification |
| Planning | Occupancy grid, inflation, A\*, path smoothing, waypoints; tested standalone | **Dynamic re-planning (the "wow" feature)** |
| Control | Waypoint follower with PID heading control; metrics | PID tuning, trajectory smoothing, speed profiles |
| Decision | FSM: IDLE→FIND_PACKAGE→NAVIGATE→PICK(placeholder)→PLAN_DELIVERY→DELIVER→DONE | PICK real (attach), re-plan transitions, full mission report |
| Integration | **Robot drives an A\*-planned path in Webots; vision + LiDAR run live; FSM skeleton runs end-to-end** | Full mission incl. dynamic obstacle |
| Extras | — | Dashboard (Webots `Display`), second robot, Dijkstra/RRT comparison, etc. |

**Rule:** do not build final-eval features now, but design interfaces so they can be added without refactoring (see §15).

---

## 3. What the mid-review needs from us

6–8 slide PPT, 5 min talk + 2 min Q&A. Slides: (1) problem & objectives, (2) background & novelty, (3) methodology + block diagram, (4) simulation progress, (5) preliminary results / validation / challenges, (6) team contribution & next steps. A draft deck `IntelliBot_MidTerm_Review.pptx` already exists; **its progress/result claims must be made true or edited to match reality** once the build is done (see checklist Phase 8).

Therefore the build must produce **real evidence**: screenshots, short screen recordings, plots, console logs, and measured numbers (see §13). Evidence capture is part of every phase, not an afterthought.

---

## 4. Ground rules for the agent

1. **Webots-free core.** Put all logic that doesn't need the simulator (kinematics, PID, grid, transforms, A\*, path utils, vision detector on a NumPy image, FSM logic) in plain-Python modules that import neither `controller` nor Webots. Test them with `pytest` and offline scripts. Only `intellibot_controller.py` and `perception.py` (device wrappers) touch the Webots API.
2. **One source of truth for numbers.** All constants (robot dims, grid resolution, thresholds, world layout, device names, colors) live in `config.py`. No magic numbers elsewhere.
3. **World is generated from config.** `tools/generate_world.py` writes `worlds/warehouse.wbt` from the layout in `config.py`, so the occupancy grid and the simulated world can never disagree. (A runtime consistency check via supervisor is a P1 task.)
4. **Verify, don't assume.** Several Webots conventions (lidar ray order, camera axes, coordinate system) must be *measured* on the installed version (§12). Record findings in `docs/webots_findings.md`.
5. **Simple > clever.** Prefer clear 30-line functions with docstrings over abstractions. The team has to explain this code.
6. **Small verified steps.** After each checklist item: run its "Done when" check, tick the box, add 1–3 lines to `docs/progress_log.md`.
7. **Never edit the Webots sample controllers** in `controllers/` (they are reference material). Read them; copy snippets into our modules.
8. **Log everything useful** to `logs/*.csv` + console (formats in §13). Logs feed the results slide.
9. **Don't fake results.** If something doesn't work yet, say so in the progress log and the final report. A documented limitation beats an invented number.
10. **Priorities:** `[P0]` must-have for mid-eval, `[P1]` should-have, `[P2]` nice-to-have. If time runs short, finish P0 only.
11. **If you cannot run Webots yourself:** write the code, then give the human the exact command/steps to run and state precisely what output to paste back. Never claim a simulation result you did not observe.

---

## 5. Tech stack & environment

- **Webots** (R2023b or newer; check installed version and note it in `docs/webots_findings.md`).
- **Python 3.9+** with: `numpy`, `opencv-python`, `matplotlib`, `pytest`. (`requirements.txt` at repo root.) In Webots: *Preferences → General → Python command* must point to the interpreter that has these packages (on Windows, the full path to `python.exe` or the venv's).
- Webots Python API uses **camelCase** (`getRangeImage`, `setVelocity`) — same names as the C/C++ docs. Therefore the C files/Makefiles in the sample folders are only useful as *API usage references*; use the `.py` samples.
- Editor: VS Code. Optional: git, one commit per milestone.

Useful CLI (if the agent can run Webots):
```
webots --batch --mode=fast --stdout --stderr worlds/warehouse.wbt
```
Do **not** add `--no-rendering` for any test that needs the camera. A Supervisor controller can end a run by calling `robot.simulationQuit(0)`.

---

## 6. Frames, units, conventions

- Units: **meters, radians, seconds**. Convert to degrees only when printing.
- Use Webots **ENU** world (`WorldInfo.coordinateSystem "ENU"`): x east, y north, **z up**; ground plane is x–y. Robot yaw θ is rotation about +z, 0 = facing +x, positive = counter-clockwise (left).
- Robot frame follows FLU: x forward, y left, z up.
- **World frame** origin = arena center.
- **Grid frame:** cell `(row, col)`; resolution `GRID_RES = 0.1 m`. World→grid: `col = floor((x - x_min)/res)`, `row = floor((y - y_min)/res)`; grid→world returns the **cell center**. Implement once in `mapping.py` and reuse everywhere.
- Transform robot→world (used for lidar hits, camera detections):
  ```
  x_w = x_r + d*cos(θ + φ)
  y_w = y_r + d*sin(θ + φ)      # d = range, φ = bearing in robot frame (left = +)
  ```
- Wrap all angles with `wrap_to_pi(a) = atan2(sin a, cos a)`.

---

## 7. World specification (`worlds/warehouse.wbt`)

**Arena:** 8 m × 6 m (x∈[−4,4], y∈[−3,3]) with walls. Flat floor, neutral gray. Soft, even lighting (one directional light + ambient; **disable cast shadows** to keep HSV detection stable). `basicTimeStep` 16 ms.

**Suggested starting layout** (agent may adjust, but keep constraints below). All coordinates are object centers in world frame.

| Object | DEF name | Center (x, y) | Size (x × y × z, m) | Color |
|---|---|---|---|---|
| Shelf 1 | `SHELF_1` | (−2.0, 1.9) | 1.6 × 0.5 × 1.0 | dark gray |
| Shelf 2 | `SHELF_2` | (2.0, 1.9) | 1.6 × 0.5 × 1.0 | dark gray |
| Shelf 3 | `SHELF_3` | (−2.0, −1.9) | 1.6 × 0.5 × 1.0 | dark gray |
| Shelf 4 | `SHELF_4` | (2.0, −1.9) | 1.6 × 0.5 × 1.0 | dark gray |
| Static crate 1 | `OBST_1` | (0.0, 0.6) | 0.4 × 0.4 × 0.4 | brown/wood |
| Static crate 2 | `OBST_2` | (−0.8, −0.9) | 0.4 × 0.4 × 0.4 | brown/wood |
| Pickup station (floor patch, no collision) | `PICKUP_AREA` | (−3.0, 0.0) | 1.2 × 1.6 × 0.01 | light gray/white |
| Package P1 | `PKG_P1` | (−3.1, 0.5) | 0.2³ cube | **red** (pure, saturated) |
| Package P2 | `PKG_P2` | (−3.1, 0.0) | 0.2³ cube | **green** |
| Package P3 | `PKG_P3` | (−3.1, −0.5) | 0.2³ cube | **blue** |
| Delivery Zone A (floor patch) | `ZONE_A` | (3.0, 0.9) | 1.0 × 1.0 × 0.01 | yellow |
| Delivery Zone B (floor patch) | `ZONE_B` | (3.0, −0.9) | 1.0 × 1.0 × 0.01 | cyan |
| Robot start | `INTELLIBOT` | (0.0, −0.3) heading 0 | — | — |

**Layout constraints (must hold):**
- Every aisle ≥ **1.0 m** clear (robot radius 0.18 m + safety 0.07 m, inflated both sides).
- There must be **at least two topologically distinct routes** between the pickup area and Zone B (e.g., center aisle and an outer corridor) — needed for the final-eval re-planning demo. Verify with A\* by temporarily blocking one route.
- Package colors are pure **red / green / blue**; delivery-zone colors (yellow / cyan) and shelf/floor colors must stay outside those hue ranges so the detector never confuses them.
- Packages are `Solid`s with `boundingObject` + `physics` (so they can be pushed/attached later), each with a `DEF`. Floor patches have no boundingObject (non-colliding).
- Wall/shelf/crate/package rectangles are all described in `config.py` as `STATIC_OBSTACLES = [(cx, cy, sx, sy), ...]` and used to build the known-map occupancy grid.

World file tip: recent Webots versions require `EXTERNPROTO` lines for any PROTO used in a `.wbt`. Either copy the header + EXTERNPROTO lines from a world shipped with the installed version, or build everything from primitive nodes (`Solid`, `Shape`, `Box`) to avoid PROTO dependencies.

---

## 8. Robot specification (`DEF INTELLIBOT`, `controller "intellibot_controller"`, `supervisor TRUE`)

Build a **custom differential-drive robot** from base nodes (this is also the cleanest way to demonstrate kinematics). If physics tuning eats too much time (robot tips, wheels slip, jitter), fall back to a Webots-shipped differential-drive PROTO (Pioneer 3-DX or TurtleBot3 Burger) and adapt the constants in `config.py` — this is a P2 fallback, decide quickly.

| Parameter | Value |
|---|---|
| Body | box 0.30 (x) × 0.20 (y) × 0.10 (z), mass ≈ 2 kg |
| Wheel radius `r` | 0.05 m |
| Wheel separation `L` | 0.24 m (between wheel centers) |
| Wheel motor max velocity | ~12 rad/s (cruise limit v ≈ 0.30 m/s, ω ≈ 1.5 rad/s) |
| Support | 2 passive casters (front/back spheres), **zero friction** via a `ContactProperties` entry |
| Robot radius for planning | 0.18 m (circumscribed) + 0.07 m margin → inflate obstacles by **0.25 m** |

**Devices and names (use exactly these strings; they live in `config.py`):**

| Device | Name | Notes |
|---|---|---|
| RotationalMotor ×2 | `left_motor`, `right_motor` | velocity mode: `setPosition(float('inf'))`, then `setVelocity(ω)` |
| PositionSensor ×2 | `left_encoder`, `right_encoder` | wheel odometry |
| Lidar | `lidar` | 1 layer, 360 rays, FOV 2π, range 0.05–4.0 m, mounted at robot center on top (z≈0.14) |
| Camera | `camera` | 320 × 240, FOV ≈ 1.0 rad, at front (x≈0.15, z≈0.12), looking forward |
| GPS | `gps` | ground-truth position for validation (see below) |
| InertialUnit | `imu` | yaw → heading (`getRollPitchYaw()[2]`) |
| (final) TouchSensor bumper | `bumper` | collision counter |
| (final) Connector / supervisor attach | — | pickup |

**Pose source policy:** the robot's *operating* pose estimate for control is GPS+IMU in the mid-eval (simple, reliable). Wheel odometry is implemented **in parallel** and compared against GPS+IMU to produce a drift/validation result. Document this honestly: GPS+IMU is idealized (simulation) localization; real robots would use odometry+LiDAR localization/SLAM (future work).

---

## 9. Software architecture

```
RI-Project/
├── worlds/
│   └── warehouse.wbt                  # GENERATED by tools/generate_world.py
├── controllers/
│   ├── (existing Webots sample folders — leave untouched)
│   └── intellibot_controller/         # Webots requires <folder>/<folder>.py
│       ├── intellibot_controller.py   # entry point: main loop, wires modules
│       ├── config.py                  # ALL constants, layout, device names
│       ├── kinematics.py              # fwd/inv kinematics, odometry, wrap_to_pi   (pure)
│       ├── pid.py                     # PID class with anti-windup, output limits   (pure)
│       ├── mapping.py                 # OccupancyGrid, transforms, inflation, lidar→world (pure)
│       ├── a_star.py                  # A* (8-connected), returns grid path         (pure)
│       ├── path_utils.py              # smoothing, grid→world waypoints, path length (pure)
│       ├── detector.py                # OpenCV HSV package detector on ndarray      (pure)
│       ├── perception.py              # Webots wrappers: LiDAR, camera, GPS/IMU     (Webots)
│       ├── waypoint_follower.py       # WaypointFollower (PID heading, speed shaping)(pure logic)
│       ├── decision_maker.py          # FSM                                         (pure logic)
│       └── logger.py                  # CSV + console logging
├── drive_test/ (optional extra controllers: e.g. controllers/drive_test/drive_test.py)
├── tools/
│   ├── generate_world.py              # config → warehouse.wbt
│   ├── plot_astar.py                  # matplotlib figure of grid + path
│   ├── plot_lidar.py                  # polar/XY plot from logged scans
│   ├── eval_detector.py               # offline detector evaluation on saved frames
│   └── analyze_logs.py                # metrics + plots from logs/*.csv
├── tests/                             # pytest: kinematics, pid, mapping, a_star, path_utils, detector, fsm
├── docs/
│   ├── samples_audit.md               # what each Webots sample offers (Phase 0)
│   ├── webots_findings.md             # measured conventions (lidar order, axes, version…)
│   ├── architecture.md                # block diagram + module explanations
│   ├── progress_log.md
│   ├── evidence/                      # screenshots, plots, gifs, frames, logs used on slides
│   ├── qa_prep.md                     # likely viva questions with answers
│   └── project_report.md
├── logs/
├── requirements.txt
├── README.md
├── checklist.md
└── project_summary.md
```

Why this differs slightly from the original README tree: Webots imports controller modules from the controller's own folder, so keeping algorithm/vision modules *inside* `intellibot_controller/` avoids `sys.path` hacks. `tests/` and `tools/` add that folder to `sys.path` explicitly.

**Data flow (one control step):**
```
sensors → perception (pose, lidar scan, camera frame → detections)
        → decision_maker (FSM: current state, goals)
        → planner (a_star on OccupancyGrid → path → waypoints)
        → controller (WaypointFollower: waypoint + pose → v, ω)
        → kinematics (v, ω → wheel speeds) → motors
        → logger
```

**Main loop skeleton:**
```python
robot = Supervisor()
dt = int(robot.getBasicTimeStep())
perception = Perception(robot, dt)
fsm = DecisionMaker(...)
while robot.step(dt) != -1:
    pose = perception.get_pose()          # x, y, theta
    scan = perception.get_lidar()         # ranges array (NaN/inf handled)
    dets = perception.get_detections()    # list[Detection]
    v, w = fsm.update(pose, scan, dets, sim_time)
    wl, wr = inverse_kinematics(v, w)
    perception.set_wheel_speeds(wl, wr)
    logger.log(...)
```

---

## 10. Algorithm specifications

### 10.1 Kinematics (`kinematics.py`)
```
Forward:  v = r/2 · (ωR + ωL)            ω = r/L · (ωR − ωL)
Inverse:  ωR = (v + ω·L/2) / r           ωL = (v − ω·L/2) / r
Clamp wheel speeds to motor max (scale both to preserve curvature).
Odometry (per step, Δφ from encoder deltas):
  ΔsR = r·ΔφR,  ΔsL = r·ΔφL,  Δs = (ΔsR+ΔsL)/2,  Δθ = (ΔsR−ΔsL)/L
  x += Δs·cos(θ + Δθ/2);  y += Δs·sin(θ + Δθ/2);  θ += Δθ
```
Handle the first PositionSensor reading (can be NaN) by initializing the previous value after the first valid step.

### 10.2 PID (`pid.py`)
Standard `PID(kp, ki, kd, out_min, out_max)` with integral clamp (anti-windup) and `dt` argument. Unit-test: step response, clamping, derivative sign.

### 10.3 Occupancy grid (`mapping.py`)
- `OccupancyGrid(x_min, x_max, y_min, y_max, res)` with `static` layer (known map from `STATIC_OBSTACLES`) and `dynamic` layer (LiDAR-detected, final eval), combined = static | dynamic.
- `inflate(radius)` using a **circular** kernel (not a square) → planning grid. Keep the raw grid; recompute inflated grid on updates.
- API (keep stable for the final eval): `world_to_cell`, `cell_to_world`, `is_free(cell)`, `add_obstacle_world(x, y, radius)`, `clear_dynamic()`, `inflated()`.
- `lidar_to_world(pose, ranges, angles)` → Nx2 hit points (ignore inf/NaN/out-of-range).
- `path_is_blocked(path_cells, grid)` — implement and unit-test now (used by final-eval re-planning).

### 10.4 A\* (`a_star.py`)
- 8-connected moves; costs 1 (orthogonal) and √2 (diagonal); **no corner cutting** (a diagonal move is allowed only if both adjacent orthogonal cells are free).
- Heuristic: octile distance. Tie-break to reduce expansions.
- Returns `(path_cells, info)` with `info = {cost, expansions, runtime_ms}`; returns `None` cleanly if no path or start/goal blocked. Snap a blocked start/goal to the nearest free cell (document it).
- Validate optimality against a Dijkstra implementation on random grids (test).

### 10.5 Path post-processing (`path_utils.py`)
- Line-of-sight (Bresenham) simplification → fewer waypoints.
- Optional: waypoint spacing/smoothing for "trajectory generation" (e.g., resample at 0.3 m spacing, or light corner smoothing that stays collision-free).
- Convert to world waypoints (cell centers); `path_length(waypoints)`.

### 10.6 Waypoint follower (`waypoint_follower.py`)
Per step, given pose and current waypoint:
```
dx, dy = wp - pos;  dist = hypot(dx, dy)
heading_err = wrap_to_pi(atan2(dy, dx) - θ)
w = PID_heading(heading_err)                  # clamp to ±W_MAX
v = V_MAX · clamp(cos(heading_err), 0, 1) · min(1, dist / SLOWDOWN_DIST)
if |heading_err| > TURN_IN_PLACE_THRESH: v = 0   # rotate first when badly misaligned
waypoint reached when dist < WP_TOLERANCE (0.10 m); last waypoint uses GOAL_TOLERANCE.
```
Also: **LiDAR safety stop** — if any ray in the forward cone (±30°) is < `STOP_DIST` (0.30 m), command v = 0 and flag `BLOCKED` (this is the hook the final-eval re-planner will use). Track cross-track error to the path segment for metrics.

### 10.7 LiDAR processing (`perception.py` / `mapping.py`)
- `getRangeImage()` → NumPy array; replace `inf`/NaN with `max_range` for processing, but keep a validity mask.
- Ray angle for index `i` must be taken from a **measured** mapping (see §12), stored in `config.LIDAR_ANGLE_SIGN` / offset. Do not assume.
- `obstacle_in_sector(scan, angle_min, angle_max, dist)` utilities; front cone / left / right sectors.

### 10.8 Package detector (`detector.py`) — pure OpenCV
1. Camera image: Webots returns **BGRA** bytes → `np.frombuffer(img, np.uint8).reshape(h, w, 4)[:, :, :3]` (BGR).
2. BGR→HSV; per-color masks (red needs two hue ranges, 0–10 and 170–180 in OpenCV's 0–180 hue scale). Tunable thresholds in `config.py` (`HSV_RANGES`).
3. Morphological open + close to clean the mask; find contours; keep the largest with area ≥ `MIN_AREA` and aspect ratio ~0.6–1.6.
4. Output `Detection(pkg_id, color, bbox, centroid, area, score)` where `score` is a **documented heuristic** (e.g., contour-area/bbox-area × size factor), *not* a statistical confidence. Say "heuristic score" on slides.
5. Annotated image (bbox + label) can be saved for evidence.

**Position estimate for a detection** (spatial transformation chain):
```
f_px   = (W/2) / tan(FOV/2)                      # horizontal FOV
φ      = -atan2(u_center - W/2, f_px)            # right-in-image → negative (left-positive robot frame)
d      = lidar range at bearing φ (min over ±3° window); fallback d = f_px·PKG_SIZE/bbox_width
(x_w, y_w) = pose + d·(cosθ+φ, sinθ+φ)
```
Compare against supervisor ground truth (`getFromDef("PKG_P3").getPosition()`) → position error metric.

### 10.9 Decision maker FSM (`decision_maker.py`)

| State | Entry / action | Exit condition → next |
|---|---|---|
| `IDLE` | load task (`pkg_id="P3"`, `zone="B"`), reset | immediately → `FIND_PACKAGE` |
| `FIND_PACKAGE` | rotate in place (ω≈0.6 rad/s) scanning ≤ 2 turns; run detector for target color | N consecutive detections → estimate world pos, add package to grid as obstacle → `NAVIGATE`; timeout → `ERROR` |
| `NAVIGATE` | choose free **approach pose** ~0.45 m from package; A\* → waypoints; follow | within tolerance → `PICK`; (final: `BLOCKED` → replan) |
| `PICK` | stop, face package, log distance. **Mid-eval: placeholder** (log "PICK placeholder"). Final: attach | done → `PLAN_DELIVERY` |
| `PLAN_DELIVERY` | A\* from current pose to Zone B center | path found → `DELIVER`; none → `ERROR` |
| `DELIVER` | follow waypoints to zone | in zone tolerance → `DONE` |
| `DONE` | stop, print mission report (time, path length, replans, collisions) | terminal |
| `ERROR` | stop, print reason | terminal |

FSM logic must be unit-testable with fake inputs (no Webots). Print every transition: `[t=12.3s] STATE: NAVIGATE -> PICK`.

---

## 11. Reusing Webots sample assets (the `controllers/` folders)

The folders in `controllers/` are Webots' own *device demo* controllers (each paired with a world in the Webots installation under `projects/samples/devices/worlds/`). **Only the folder names were visible when this document was written, not their contents** — so the agent's first task (Phase 0) is to open each relevant `.py`, record what it demonstrates, and write `docs/samples_audit.md`. Expected usefulness:

| Sample folder(s) | Use for | When |
|---|---|---|
| `lidar` | enabling the Lidar, `getRangeImage()`, point cloud, resolution/FOV queries | **Mid** — read first |
| `hokuyo`, `sick`, `sick_point_cloud` | realistic lidar PROTO models (Hokuyo UTM-30LX, SICK LMS291) | optional; plain `Lidar` node is simpler/configurable |
| `camera` | `enable`, `getImage`, `saveImage`, BGRA handling | **Mid** |
| `camera_recognition` | `Recognition` node → object list with positions/colors = **ground truth** to validate our OpenCV detector | **Mid** (validation only; the detector itself must be our own OpenCV code) |
| `camera_segmentation` | segmentation image = ground-truth mask for IoU of HSV mask | P2 |
| `camera_auto_focus` | — | skip |
| `gps` | reading position | **Mid** (pose + ground truth) |
| `inertial_unit`, `imu`, `compass`, `gyro`, `accelerometer` | heading (use `inertial_unit`) | **Mid** |
| `position_sensor`, `encoders` | wheel encoder reading → odometry | **Mid** |
| `motor`, `motor2`, `motor3`, `coupled_motors` | rotational motor velocity control pattern | **Mid** |
| `distance_sensor` | short-range sensors (e.g., for final approach to package) | P2 / final |
| `bumper` (TouchSensor) | collision counter for "Collisions: N" in report | **Final** |
| `sample_supervisor`, `gps_supervisor`, `display_supervisor` | Supervisor API: `getFromDef`, `getField`, `setSFVec3f`, reading ground truth, moving/spawning obstacles | Mid (ground truth), **Final** (dynamic obstacle, attach) |
| `connector` | active/passive connector = "magnetic attach" pickup | **Final** |
| `vacuum_gripper` | alternative pickup mechanism | **Final** (option B) |
| `display` | draw map/path/robot/state on a Webots `Display` → in-sim dashboard | P2 mid, **Final** |
| `pen` | `Pen` node paints the robot's trail on the floor → great visual of actual path | P2 mid |
| `emitter_receiver` / `EmitterReceiver` | messaging; second robot / external dashboard | optional final |
| `led` | status light per FSM state | P2 |
| everything else (`altimeter`, rotors, `battery`, `brake`, `track*`, `radar`, `range_finder`, `spherical_camera`, `skin`, `speaker*`, `linear_motor`, `hinge_joint_with_backlash`, `light_sensor`, `force*`…) | not needed | skip |

Other useful assets inside the Webots installation: PROTO robots under `projects/robots/` (Pioneer 3-DX, TurtleBot3 Burger — fallback robot), and factory/warehouse-style objects under `projects/objects/factory/` (boxes, crates, pallets; check what exists in the installed version — if no shelf PROTO exists, build shelves from `Solid` + `Box`). Use `Add node → PROTO nodes (Webots Projects)` in the GUI to browse.

---

## 12. Webots gotchas (check each; record outcomes in `docs/webots_findings.md`)

- **Controller name:** `controllers/intellibot_controller/intellibot_controller.py` — folder and main file names must match, and the robot's `controller` field must use that name.
- **Coordinate system:** confirm `WorldInfo.coordinateSystem` (use ENU). Axes of Camera/Lidar in current Webots follow FLU (look along local +x) — **verify in the docs/behavior of the installed version**.
- **Enable devices** (`device.enable(dt)`) before use; first sensor values are valid only after the first `robot.step()`. `PositionSensor` can return NaN on step 1.
- **Lidar ray order and handedness:** place an obstacle on the robot's left only, print which indices read short, and derive the angle mapping empirically. `inf` = no return.
- **Camera image:** BGRA bytes, row-major; `getImage()` returns bytes valid until next step. Copy if you keep it.
- **Camera FOV** in Webots is the *horizontal* FOV.
- **Motors:** velocity mode requires `setPosition(float('inf'))`. Respect `getMaxVelocity()`.
- **Physics:** if the robot jitters/tips, lower body height/raise mass, check `ContactProperties` for casters, use smaller `basicTimeStep` (16 or 8 ms), and set a sensible `WorldInfo.basicTimeStep`/`optimalThreadCount`.
- **Supervisor:** `supervisor TRUE` on the robot; `getFromDef("NAME")`; `getField("translation").setSFVec3f([...])`; call `resetPhysics()` after teleporting a solid.
- **`EXTERNPROTO`** requirement in newer world files (see §7).
- **Lighting/HSV:** shadows and specular highlights shift HSV values; keep lighting flat for detection.
- **`--no-rendering`** disables camera output.
- **Python path:** Webots uses the Python command in Preferences — missing `cv2`/`numpy` errors usually mean this points to the wrong interpreter.

---

## 13. Metrics & evidence to produce for the mid-eval

All logged to `logs/` and summarized by `tools/analyze_logs.py` into `docs/evidence/metrics.md`.

| Topic | Metric | How |
|---|---|---|
| Kinematics | commanded (v, ω) vs measured (from GPS/IMU finite differences): % error for ≥4 test commands | drive_test controller |
| Odometry | final position/heading error and RMS drift vs GPS on ≥3 trajectories (line, square, arc) | log both, plot overlay |
| LiDAR | measured vs true range at 0.5/1.0/2.0 m; obstacle-flag latency (steps); polar plot | known obstacle placements |
| Vision | per-color detection rate and false-positive rate over ≥30 frames at several distances/angles; lighting variants (normal/dim/bright) | saved frames + `eval_detector.py`; ground truth from `camera_recognition`-style `Recognition` or supervisor |
| Position estimate | package world-position error (m) vs supervisor ground truth | log |
| A\* | path length vs Dijkstra (must be equal cost), expansions, runtime (ms); figure with grid + inflated obstacles + path | `plot_astar.py` |
| Following | mean/max cross-track error, time to goal, success rate over N≥10 runs from different starts, goal error | `analyze_logs.py` |
| Integration | one full run log + short screen recording: FIND → NAVIGATE → PICK(placeholder) → PLAN → DELIVER → DONE | final demo run |

**Run log CSV columns:** `t, x, y, theta, x_odom, y_odom, theta_odom, v_cmd, w_cmd, wl, wr, state, wp_idx, cross_track_err, lidar_front_min, n_detections`.

**Evidence files** (save under `docs/evidence/`): world screenshot(s); robot close-up; LiDAR rays visualization; camera frame with bbox; A\* plot; trajectory overlay plot (planned vs actual); metrics table; ≤60 s screen recording of the full run.

---

## 14. Module ownership map (for later split among the 3 humans)

| Person | Area | Modules / files |
|---|---|---|
| **A — Simulation & Control** | world, robot, kinematics, motion control | `generate_world.py`, `warehouse.wbt`, robot definition, `kinematics.py`, `pid.py`, `waypoint_follower.py`, drive tests |
| **B — Perception** | LiDAR, camera, vision, validation | `perception.py`, `detector.py`, `eval_detector.py`, `plot_lidar.py`, detector/lidar tests |
| **C — Planning & Decisions** | grid, A\*, FSM, integration | `mapping.py`, `a_star.py`, `path_utils.py`, `decision_maker.py`, `intellibot_controller.py`, `plot_astar.py` |

Each person should be able to explain, in `docs/architecture.md`-level detail, their modules **and** the interfaces to the other two. Put a short "How it works / how to explain it" section per module in `docs/architecture.md`, and prepare viva Q&A in `docs/qa_prep.md`.

---

## 15. Hooks to leave for the final evaluation

- `OccupancyGrid.add_obstacle_world()`, `clear_dynamic()`, `path_is_blocked()` exist and are tested → dynamic re-planning only needs wiring.
- WaypointFollower exposes a `status` (`FOLLOWING`, `BLOCKED`, `REACHED`) and the FSM has a defined slot for a `REPLAN` transition.
- Perception returns structured data (pose, scan, detections) — a Display/dashboard can read it.
- `Supervisor` access is already available in the main controller, so spawning a dynamic obstacle (move a pre-placed `DEF DYN_OBSTACLE` into the route at a chosen time) and attaching a package (follow-pose or Connector) only needs new code in dedicated functions.
- Config has placeholders: `DYNAMIC_OBSTACLE_TRIGGER_TIME`, `PICK_DISTANCE_THRESHOLD = 0.18`.
- Mission report generator prints replans/collisions even if they are 0/N-A now.

---

## 16. Definition of "mid-eval ready"

1. `worlds/warehouse.wbt` opens, robot drives, no physics glitches.
2. Unit tests pass (`pytest` green).
3. Live run: robot finds P3 with the camera, plans with A\*, drives to the approach pose, runs placeholder pick, plans to Zone B, delivers, prints a mission report — with FSM transitions logged.
4. All P0 metrics in §13 measured, evidence saved.
5. Deck updated to match the real results; README with run instructions; `docs/qa_prep.md` ready.
