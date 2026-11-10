# Implementation plan & status — Ankit (branch `feature/ankit-arm-shelves-camera`)

Scope: **Phases 11, 16, 17 and 18 of `checklist_v2.md`** plus three additional
requirements: **(B) a working robotic arm**, **(C) destination shelving**,
**(D) front/rear camera fixes**.

Status words used below (no item is called "done" unless it was verified):

* **Implemented + offline-verified** — code written, covered by pytest and/or the
  offline kinematic mission simulator (`tools/offline_mission_sim.py`, NOT physics).
* **Pending Webots** — needs a run on the GPU laptop (commands in the README).
* **Not implemented** — honest gap, with the reason.

## 0. How the phase numbers were reconciled

`checklist.md` (mid-eval) stops at Phase 9 (backlog). The numbering that matches
"Phase 11, 16, 17, 18" is `checklist_v2.md` (it says it supersedes `checklist.md`):

| Phase | Title in checklist_v2.md |
|---|---|
| 11 | Kalman filtering & sensor fusion |
| 16 | Full mission integration & scenario suite |
| 17 | Dashboard, visualization, demo assets |
| 18 | Evaluation & ablations |

Phases 11/16 overlap with the arm work through items **11.9** ("fusion in
manipulation checks") and **16.1/16.2** ("FSM v2 … pick → navigate → dock →
place on assigned slot → confirm → report").

## 1. What the repository looked like (audit, 2026-11-08, `main` @ 6a0f76c)

* Live mission = `worlds/warehouse.wbt` (8 × 6 m, v1 robot 0.30 × 0.20 m,
  `intellibot_controller`). **No arm**: PICK was a timed pause and the parcel was
  teleported onto the robot by the Supervisor (`PackageCarrier`).
* `arm_kinematics.py`, `manipulation.py`, `trajectory.py` existed as **pure design
  studies** for a v2 lift + SCARA robot (`warehouse_v2.wbt`), never wired. That
  v2 world used the non-existent node `LinearJoint` and `coordinateSystem "NUE"`
  with z-up data, so it could not load correctly.
* Cameras: the last commit had set `CAMERA_FOV = REAR_CAMERA_FOV = 3.14159265`
  (labelled "90 deg") — Webots limits a **planar** camera to `0 < FOV ≤ π`; at π
  the focal length is ~0 px and the image collapses. The committed `.wbt` files
  had been edited to 1.5708 / z = 0.16 m, so **world and config disagreed**
  (regenerating the world from config produced broken cameras). The rear camera's
  housing box was placed **in front of its own lens** (+0.01 m in its rotated
  frame). Mount offsets (+0.01 / +0.04) were hard-coded in the generator.
  The detector thresholds had been loosened (30 px², aspect 0.4–2.5) to cope,
  which broke `tests/test_detector.py::test_small_blob_rejected`.
* `pytest`: **116 passed, 2 failed** before any change (the detector test above
  and `test_live_replanning_with_grid`, whose log line had been removed).

## 2. Requirement → implementation map

### Phase 11 — sensor fusion

| Req. | Existing | Change made | Files | Acceptance | Status |
|---|---|---|---|---|---|
| 11.5 camera landmark update, R scaled by light | EKF had GPS/IMU/landmark updates only | `EKFLocalizer.update_bearing` (bearing-only, Joseph form, χ² gate) + `camera_bearing_variance(brightness)`: R × (nominal/mean)², ∞ when dark | `ekf_localization.py`, `camera_pipeline.py`, `config.py` | lit camera corrects heading; dim camera smaller gain; dark camera no effect; outlier gated | Implemented + offline-verified (`tests/test_camera_fusion.py`). **Not wired**: the v1 world has no known visual landmark yet |
| 11.9 fusion in manipulation checks | abstract dict FSM in `manipulation.py` | "parcel held" = finger width ✓ AND palm range ✓ AND (pad force ✓ OR latch locked); "released" = palm range after back-off ≈ expected AND latch open AND fingers open; "slot blocked" = joint tracking error | `gripper.py`, `pick_place.py` | every combination tested; no success without sensor confirmation | Implemented + offline-verified; pending Webots |
| 11.7 / 11.8 fault experiments, Q/R tuning | — | not in this branch | — | — | Not implemented (needs Webots runs) |

### Phase 16 — mission integration

| Req. | Existing | Change made | Files | Acceptance | Status |
|---|---|---|---|---|---|
| 16.1 FSM v2 | v1 FSM + interactive loop | new states `ARM_STOW, DOCK_PICK, ARM_PICK, VERIFY_ATTACHMENT, DOCK_PLACE, ARM_PLACE, VERIFY_DELIVERY, ARM_RECOVER`; retries; errors never reach DONE; base (0,0) whenever the arm moves; legacy path kept when no arm exists | `decision_maker.py` | 16 FSM tests incl. every failure branch | Implemented + offline-verified |
| 16.2 S1 store mission | placeholder | full pick → dock → place on assigned slot → verify → report | `decision_maker.py`, `docking.py`, `pick_place.py`, `shelving.py`, `intellibot_controller.py` | offline kinematic: 10/10 campaign start poses DONE, parcel ≤ 0.5 cm from slot centre | Offline-verified; **≥ 8/10 Webots runs pending** |
| 16.3 S2 retrieve | — | building blocks reusable (pick at any height, place on floor tested) | — | — | Not implemented |
| 16.4 scenario suite | — | S1 variants + camera/arm checks in `run_scenario.py` | `tools/run_scenario.py` | — | Partial (S2–S8 not in this branch) |
| 16.5 mission report | text report | + result, slot, attempts, docking errors, manipulation checks, failure reason; JSON file; Supervisor ground-truth audit (evaluation only) | `decision_maker.py`, `intellibot_controller.py` | report written at DONE/ERROR (smoke test) | Implemented; pending Webots |
| 16.6 reproducibility | — | env-driven task (`INTELLIBOT_TASK_*`), per-run log folders, batch CLI | `tools/run_scenario.py`, `intellibot_controller.py` | dry-run tested | Implemented; pending Webots |

### Phase 17 — visualization

| Req. | Change made | Status |
|---|---|---|
| 17.2 LED = FSM state | colours for the 8 new states (`config.STATE_LED`) | Implemented |
| 17.3 analysis plots | `tools/plot_arm_workspace.py` (design figure + table), `tools/plot_manipulation.py` (joint/width/force/latch timeline) | Implemented; offline figures in `docs/evidence/`; Webots figures pending |
| 17.1 / 17.4 / 17.5 | unchanged (dashboard already shows the state string); feature freeze is a team decision | — |

### Phase 18 — evaluation

| Req. | Change made | Status |
|---|---|---|
| 18.1 experiment matrix | `docs/experiments.md` (manipulation, camera, docking rows; N, seeds, metrics) | Written |
| 18.2 run the matrix | — | Pending Webots |
| 18.3 analysis | `tools/analyze_manipulation.py` → `docs/evidence/manipulation_results.md` (all runs, sensor-vs-ground-truth agreement) | Implemented (tested on synthetic reports) |
| 18.4 failure analysis | failure taxonomy = the sequencer's failure reasons; offline-observed failure documented (oblique LiDAR corner fit, fixed) | Framework written; Webots data pending |
| 18.5 statistical sanity | rules in `docs/experiments.md`; the tool reports every run | Written |

### (B) Robotic arm

| Requirement | Implementation | Verified by |
|---|---|---|
| physically attached | `HingeJoint` chain under the `Robot` node, on a post fixed to the base | world validator, smoke test |
| shoulder, elbow, wrist, gripper | 3 × `HingeJoint` + `RotationalMotor` + `PositionSensor` (axis 0 −1 0); 2 × `SliderJoint` + `LinearMotor` + `PositionSensor` fingers; force `TouchSensor` pads; palm `DistanceSensor`; `Connector` latch | `tests/test_world_generation.py` |
| limits, velocities, physics | joint limits (soft `minPosition/maxPosition` inside hard `minStop/maxStop`), maxVelocity/maxTorque/maxForce, link masses + COM, counterweight via base COM | world tests, tipping test |
| avoid collisions | 2-D clearance model (links + held parcel vs body, bumpers, camera housings, post, floor, rack) checked along every planned path | `tests/test_arm_pitch_kinematics.py` |
| reach parcel and shelf | IK for floor parcel and rack L1/L2 (L3 deliberately unreachable) | same |
| modular joint control | `arm_kinematics.PlanarPitchArm`, `arm_controller.ArmController`, `arm_driver.WebotsArmDriver` | `tests/test_arm_controller_gripper.py` |

### (C) Destination shelving

| Requirement | Implementation |
|---|---|
| warehouse-style rack, multiple levels | `DEST_RACK_A`, `DEST_RACK_B` against the east wall behind zones A/B; 6 uprights, 3 platforms, kick plate, back panel; 2 bays × 3 levels |
| collision + physics | static `Solid` with a `boundingObject` group (like the existing shelves) |
| defined placement locations | `shelving.Slot` ids `RA/RB-L<level>-B<bay>`; slot markers on the front edge |
| dimensions vs arm | L1 = 0.15 m, L2 = 0.41 m reachable; L3 = 0.67 m out of reach (documented) |
| in the planner's map | `config.DEST_RACK_FOOTPRINTS` appended to `STATIC_OBSTACLES`; tests: predock poses free + reachable, two routes still exist |
| parcel physically on the shelf | place → unlatch → open → back off → palm range check; Supervisor audit in the report |

### (D) Cameras

| Problem found | Fix | Test |
|---|---|---|
| planar FOV = π (degenerate) | FOV 1.0 rad for both cameras (the value of all detector evidence), `projection "planar"` explicit | `test_planar_fov_is_valid_for_both_cameras`, world validator |
| world ≠ config (hand-edited .wbt) | worlds regenerated; test fails if a world on disk differs from the generator | `test_worlds_on_disk_are_up_to_date` |
| rear housing in front of its lens | housing behind the lens for both cameras, sizes in config | `test_generated_cameras_match_config` |
| hard-coded mount offsets | `CAMERA_MOUNT_X/Z`, `REAR_CAMERA_MOUNT_X`, `REAR_CAMERA_YAW` used verbatim | same |
| fisheye heuristic for FOV ≥ 2 rad | projection chosen explicitly (`planar` → pinhole) | front/rear projection round-trip tests |
| `getImage()` None before first sample crashed `reshape` | `camera_pipeline.bgra_to_bgr` validates; invalid frames are skipped | `test_missing_frames_do_not_crash_detection` |
| no blank/frozen detection | `FrameHealthMonitor` per camera; transitions printed | `test_make_frame_flags_invalid_blank_and_frozen` |
| feed interchange risk | `CameraChannel` binds device name → role; role travels with frame and detection | `test_front_and_rear_frames_come_from_their_own_devices` |
| arm occluding cameras | lenses at 0.12 m, arm stowed above/behind them | `test_stowed_arm_and_parcel_do_not_occlude_either_camera` |
| evidence | `controllers/manip_check` (mode `cameras`) saves both feeds + Recognition comparison to `docs/evidence/cameras/` | **Pending Webots** (no images were invented) |

## 3. Files

New: `shelving.py`, `docking.py`, `arm_controller.py`, `arm_driver.py`, `gripper.py`,
`pick_place.py`, `camera_pipeline.py` (controller folder); `controllers/manip_check/`;
`tools/offline_mission_sim.py`, `tools/validate_world.py`, `tools/plot_arm_workspace.py`,
`tools/plot_manipulation.py`, `tools/analyze_manipulation.py`, `tools/run_scenario.py`;
10 new test files (+ helper `tests/arm_fakes.py`); `worlds/warehouse_manip_check.wbt`; docs listed in the README.

Modified: `config.py`, `decision_maker.py`, `perception.py`, `estimation.py`,
`mapping.py`, `arm_kinematics.py` (new class appended), `ekf_localization.py`
(method appended), `logger.py`, `waypoint_follower.py` (one attribute),
`intellibot_controller.py`, `drive_test.py` (holds the arm stowed),
`tools/generate_world.py`, regenerated worlds, `tests/conftest.py`.

## 4. Acceptance-test status

| Check | Result |
|---|---|
| `python -m pytest tests/ -q` | **244 passed** (was 116 passed + 2 failed; 126 new tests) |
| `python tools/validate_world.py` | all generated worlds OK; generator deterministic |
| offline kinematic mission, 10 campaign starts | 10/10 DONE, parcel in slot |
| Webots: world opens with 0 warnings | pending |
| Webots: `manip_check` cameras / joints / gripper / pick | pending |
| Webots: S1 mission ≥ 8/10 | pending |
