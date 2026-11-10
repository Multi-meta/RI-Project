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
inf, measured ray-angle mapping via `LIDAR_ANGLE_SIGN`, self-filter for the
arm post), the FRONT and REAR cameras as two `CameraChannel`s (each bound to
its role; preprocessing in `camera_pipeline.py`), detections tagged with the
camera they came from, and the supervisor world/config consistency check
(now including the destination racks).
**Interface:** `get_pose()`, `get_lidar()`, `get_camera_frames()`,
`get_detections(front, rear)`, `estimate_package_position()`,
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
PLAN_DELIVERY→DELIVER→DONE (+ERROR) when no arm is present; with the arm the
manipulation states of the next section replace the placeholder. Every
transition printed. Testable without Webots — planner, estimator and the
manipulator are injected.
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

## Mobile manipulator, destination racks and cameras (final evaluation)

### Robot with arm (side view, robot frame FLU)

```
             elbow (folded up when carrying)
               o
              /|          upper arm = 2 side plates (clevis), forearm passes between them
     shoulder o |
   (z=0.25 m) | o wrist ── palm ─┬─ fingers (SliderJoint x2, force pads)
              |   [parcel]       └─ Connector "gripper_latch" (active) + DistanceSensor "palm_range"
          post|
   rear cam ■─┴──────────[ body 0.30 x 0.20 ]──────────■ front cam   (lenses z = 0.12 m)
   ---- LiDAR plane z = 0.14 m (the post is the only self-return: self-filter < 0.15 m) ----
```

* **Joints:** shoulder, elbow, wrist = `HingeJoint` + `RotationalMotor` +
  `PositionSensor`, axis `0 -1 0` so a positive angle lifts the link; fingers
  = `SliderJoint` + `LinearMotor` + `PositionSensor`. Gripper pitch
  φ = q1+q2+q3 is kept 0 (horizontal) — the base supplies yaw, the arm supplies
  reach and height.
* **Geometry (config §13):** L1 = 0.28 m, L2 = 0.25 m, wrist→parcel centre
  0.16 m; joint limits inside Webots hard stops; counterweight in the base COM
  (static tipping margin ≈ 7 cm with a 0.5 kg parcel at full reach).
* **Stow / carry pose:** parcel held over the body at z ≥ 0.18 m — above the
  LiDAR plane, outside both camera frusta, within the bumper line (+1.5 cm).

### Data flow during manipulation

```
 DecisionMaker ──(when)──► Docker (docking.py) ──(v, ω)──► wheels      [arm stowed]
      │                      ▲ LiDAR range, LiDAR outline fit, pose
      └──(how)──► PickPlaceController (pick_place.py) ── steps ──┐      [base (0,0)]
                     │                                             │
          ArmController (arm_controller.py)        Gripper (gripper.py)
          IK + quintic / Cartesian lines           open / close (force-limited),
          tracking-error monitor                   latch, grasp fusion
                     │                                             │
                     └─────────── WebotsArmDriver (arm_driver.py) ─┘  ← only Webots-facing file
```

| Module | Role | Owner (suggested) |
|---|---|---|
| `arm_kinematics.PlanarPitchArm` | analytic FK/IK (elbow-up/down), limits, reach ratio; 2-D clearance model | A |
| `arm_controller.py` | joint/Cartesian motions, velocity limits, tracking-fault detection | A |
| `gripper.py` | fingers + latch + "holding" fusion (width, palm range, force/latch) | A + B |
| `pick_place.py` | named step sequences with a sensor check per step, recover | A + C |
| `docking.py` | LiDAR face range, box centre/yaw fit, coarse-to-fine docking | A + B |
| `shelving.py` | racks, slots, dock lines, robot-frame targets, ground-truth audit | C |
| `camera_pipeline.py` | BGRA→BGR, frame health (blank/frozen), light-scaled camera R | B |

### FSM with the arm

```
IDLE → ARM_STOW → FIND_PACKAGE → NAVIGATE → DOCK_PICK → ARM_PICK → VERIFY_ATTACHMENT
     → PLAN_DELIVERY (choose free slot) → DELIVER → DOCK_PLACE → ARM_PLACE → VERIFY_DELIVERY
     → DONE | RETURN_TO_CENTER (interactive)
failures → ARM_RECOVER → (retry DOCK_PICK | next slot via PLAN_DELIVERY | ERROR)
```

| Step | Sensor evidence required to continue |
|---|---|
| DOCK_PICK / DOCK_PLACE | LiDAR face range within ±0.8 cm of the standoff, lateral ≤ 1.5 cm, heading ≤ 3° |
| ARM_PICK | palm range ≤ 4 cm before closing; fingers stalled at parcel width ±1.5 cm; latch peer present → locked |
| VERIFY_ATTACHMENT | after a 5 cm lift and again in the carry pose: width ✓, palm ✓, force or latch ✓ |
| ARM_PLACE | INSERT completes without joint tracking error (else SLOT_BLOCKED → next slot) |
| VERIFY_DELIVERY | latch open, fingers open, palm range after a 6 cm back-off = 7 ± 2 cm |

### Attachment mechanism (why a Connector)

Webots' `Connector` is the documented mechanism for "an object equipped with
a passive connector grasped by a robotic gripper equipped with an active
connector". Each parcel carries four passive connectors (one per vertical
face, x-axis outward, `snap FALSE` so the parcel keeps its pose); the palm
carries the active one. `lock()` creates a real physical joint only when the
peer is present (distance ≤ 4 cm, axes opposed within 0.35 rad); `unlock()`
removes it and the parcel drops the last 3 mm onto the shelf under gravity.
The fingers still close force-limited on the parcel: they centre it and their
encoders/force pads are the independent check that something parcel-sized is
really held. No Supervisor teleport is used when the arm is present; the
Supervisor only AUDITS the final parcel position for the report.

### Destination racks

Two racks (`DEST_RACK_A/B`) against the east wall behind zones A/B; front
face at x = 3.66 m, 0.32 m deep, 2 bays × 3 levels (tops 0.15 / 0.41 /
0.67 m). Levels 1–2 are robot slots; level 3 is overflow storage beyond the
arm's reach. The kick plate in front of level 1 sits in the LiDAR plane, so
docking measures the exact distance to the rack. Footprints are part of
`STATIC_OBSTACLES`, i.e. of the planner's map; the pre-dock poses are outside
the inflated area and docking covers the last 0.2 m at ≤ 6 cm/s.

### Cameras

```
Camera "camera" (front)      planar, FOV 1.0 rad, 320x240, lens (0.15, 0, 0.12), yaw 0
Camera "rear_camera" (rear)  planar, FOV 1.0 rad, 320x240, lens (-0.15, 0, 0.12), yaw pi
getImage() BGRA ──► camera_pipeline.bgra_to_bgr (drop alpha; no swap, no flip)
            ──► FrameHealthMonitor (BLANK / FROZEN) ──► detector.detect_colors(is_rear)
            ──► estimation.bearing_from_detection: front phi = -atan(u'/f),
                                                    rear  phi = pi - atan(u'/f)
```

## Navigation stack v2: EKF pose, dynamic layer, replanning, speed governor

```
encoders ─► Odometry (signed ds, dθ) ─┐
GPS, IMU ─────────────────────────────┴─► LocalizationManager (EKF + health + kidnap recovery)
                                                │ pose, status, pose_jump
LiDAR ─► DynamicObstacleMap (evidence, observed-free clearing, moving-cell memory)
                                                ▼
                 DecisionMaker._nav_monitor ── triggers ──► A* from CURRENT pose (safe start)
                         │                                   └─► RecoveryBehavior (sim clock)
                         ▼
               WaypointFollower ─► SpeedGovernor (v_cap = min(cruise, clearance, health, zone))
                                          ─► inverse_kinematics ─► wheels
```

### localization.py — [Ankit]
EKF is the navigation pose. Initialised from the first valid GPS+IMU sample,
predicted from signed encoder increments with motion-dependent Q, GPS/IMU trust
→ R inflation, χ² gating; consistent rejected GPS for 0.5 s ⇒ relocalise and
report the jump (robot was displaced). Modes ekf | odom | gps for ablations.

### dynamic_map.py — [Ankit]
Writes `OccupancyGrid.dynamic`. Hit = +1 evidence, ray passing through = −1,
occupied at 2; hits on the static map ignored; unobserved evidence kept (memory
of a blocked route) until 30 s; vacated cells remembered 2–6 s to recognise
moving obstacles.

### safety.py — [Ankit]
`d_stop(v) = v²/(2a) + v·t + margin`; swept-corridor clearance in the direction
of travel; `SpeedGovernor` scales (v, w) together (curvature kept), ramps,
emergency-stops at the margin, caps for carrying / approach / degraded sensors;
docking runs in DOCK mode.

### decision_maker.py additions — [Ankit]
`_nav_monitor` (route-corridor check, moving-obstacle yield, lane crossing,
deviation, stuck, collision, pose jump, cooldown), `_replan` (always from the
current pose to the same goal, fresh follower), `_recovery_step`
(wait → rotate if safe → back up only with a verified rear → replan, bounded),
exact (0, 0) in every arm state, governor applied to every command.

## Tools (offline, no Webots needed)

| Tool | Output |
|---|---|
| `tools/generate_world.py` | `worlds/warehouse*.wbt` (world = config, always) |
| `tools/validate_world.py` | static checks of the generated worlds (node types, devices, FOV, DEFs, determinism) |
| `tools/offline_mission_sim.py` | kinematic (NOT physics) end-to-end run of the real FSM + arm code |
| `tools/plot_arm_workspace.py` | `docs/evidence/arm_workspace.png/.md` (design geometry) |
| `tools/plot_manipulation.py` | joint/gripper/latch timeline from `logs/manip*.csv` |
| `tools/analyze_manipulation.py` | `docs/evidence/manipulation_results.md` from mission reports |
| `tools/run_scenario.py` | batch Webots runs (S1 variants, camera/joint/gripper/pick checks) |
| `tools/plot_astar.py` | `docs/evidence/astar_pickup_to_zoneB.png` + A*/Dijkstra comparison + two-route check |
| `tools/plot_lidar.py` | `docs/evidence/lidar_scan.png` (needs `logs/lidar_scan.csv` from sim) |
| `tools/eval_detector.py` | `docs/evidence/detector_eval.md` + annotated example (needs saved frames) |
| `tools/analyze_logs.py` | `docs/evidence/metrics.md` + odometry overlay plot |
