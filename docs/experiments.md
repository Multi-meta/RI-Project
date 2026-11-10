# Experiment matrix (checklist_v2 Phase 18) — manipulation, docking, cameras

Scope of this file: the experiments for the arm / shelving / camera work of
branch `feature/ankit-arm-shelves-camera`. The fusion, navigation and
multi-robot experiments of Phases 11–15 belong to their owners and should be
added as further sections in the same format.

**Status: planned. No Webots numbers exist yet** — every "result" cell is
filled from logs by `tools/analyze_manipulation.py`, never by hand.

## 18.1 Matrix

| ID | Scenario (tools/run_scenario.py) | Configuration | N | Seeds / variation | Metrics (source) |
|---|---|---|---|---|---|
| M1 | `S1_store_P3_B` | arm + docking (this branch) | 10 | `CAMPAIGN_STARTS` 0–9 via `RUN_INDEX` | end-to-end success, time, grasp/place attempts, failure reasons (`mission_report*.json`) |
| M2 | `S1_store_P1_A`, `S1_store_P2_B` | same | 5 each | start index 0–4 | as M1 — different parcel/rack |
| M3 | M1 with RB-L1 slots pre-filled (place two parcels first, interactive) | level-2 placement | 5 | — | place success on L2, placement error |
| M4 | `pick_check` | isolated pick/carry/release, robot teleported with 5 cm / 3° offsets | 10 | offsets ± | docking errors, grasp success, parcel ground-truth trajectory (`pick_check.md`) |
| M5 | `joint_check` | joint step ±0.3 rad | 1 (deterministic) | — | final error, overshoot, settle time (`joint_check.md`) |
| M6 | `gripper_check` | open/close empty | 3 | — | width, pad forces |
| M7 | `camera_check` | 4 headings × 2 cameras | 1 per lighting level | normal / dim / bright (light intensities as in capture_set) | recognised vs detected ids, pixel offsets, health (`camera_validation.md`) |
| A1 | ablation: docking OFF (pick straight from the follower's stop pose) | `ARM_DOCK_*` bypass (temporary flag) | 10 | as M1 | grasp success with vs without docking |
| A2 | ablation: latch OFF (fingers only) | skip `LATCH` step | 10 | as M1 | ATTACHMENT_LOST rate with vs without latch |

A1/A2 need a small code switch that is NOT in this branch (deliberately: no
untested feature flags); add it only when the experiment is run.

## 18.3 Analysis

```bash
python tools/analyze_manipulation.py --logs logs/final/S1_store_P3_B/run_00   # per run folder
python tools/plot_manipulation.py logs/final/S1_store_P3_B/run_00/manip.csv
```

Report for every group: success count / N (not only a percentage),
mean ± std of time and docking errors, the failure-reason table, and the
number of runs where the robot's sensor verdict disagreed with the Supervisor
ground-truth audit (target: 0; any disagreement must be explained).

## 18.4 Failure analysis — taxonomy

The sequencer's failure reasons ARE the taxonomy, so every failed run is
classified automatically:

| Category | Reasons (prefix in `failure_reason`) | Detection |
|---|---|---|
| perception / docking | `DOCK_PICK PARCEL_FACE_NOT_FOUND_BY_LIDAR`, `LATERAL_ERROR`, `HEADING_ERROR`, `TOO_CLOSE`, `TIMEOUT` | LiDAR fit, pose vs dock line |
| reach | `PARCEL_OUT_OF_REACH`, `SHELF_TARGET_UNREACHABLE`, `JOINT_LIMIT` | IK before any motion |
| grasp | `PARCEL_NOT_AT_PALM`, `GRASP_MISSED`, `GRASP_UNVERIFIED`, `LATCH_PEER_NOT_PRESENT`, `LATCH` timeout | palm range, finger width, latch presence |
| attachment | `ATTACHMENT_LOST`, `ATTACHMENT_LOST_DURING_STOW`, `PARCEL_LOST_IN_TRANSIT` | width + palm + force/latch fusion |
| placement | `SLOT_BLOCKED`, `UNLATCH` timeout, `PARCEL_FOLLOWED_GRIPPER`, `PARCEL_NOT_IN_SLOT`, `NO_FREE_SLOT` | joint tracking error, palm range after back-off |
| hardware | `TRACKING_ERROR`, `INVALID_JOINT_SENSOR`, `SETTLE_TIMEOUT`, gripper `TIMEOUT` | position sensors |

Observed so far (offline kinematic simulator only, not Webots):

| Failure | Cause | Fix |
|---|---|---|
| `PARCEL_NOT_AT_PALM` from campaign start 1 | oblique LiDAR view of a parcel corner → single-line fit gave a 134° dock heading | box centre+yaw fit with corner detection, aisle-face selection, second refinement from the square-on pre-dock pose → 10/10 |
| robot looping between "yield" and "cleared" | follower blocked while REVERSING, blocked-handler checked only the front sector | check the sector in the direction of travel |

## 18.5 Statistical sanity rules

1. Every run that was started is reported (the analysis tool reads all
   `mission_report*.json`); a run may only be excluded for a documented
   infrastructure reason (Webots crash before IDLE), listed with its log.
2. Report counts (k/N) with the rate; N ≥ 10 for any claimed success rate.
3. Seeds/start poses are fixed (`CAMPAIGN_STARTS`, scenario names) so runs are
   repeatable; each run writes into its own `logs/final/<scenario>/run_XX/`.
4. Offline kinematic results are labelled as such and never mixed with
   Webots results in one table.

---

# Navigation / localisation / safety ablations (branch `fix/ankit-ekf-navigation-manipulation`)

Offline numbers come from `python tools/run_experiments.py`, which runs the
mission on the **kinematic** simulator and writes every value it reports to
`docs/evidence/experiments/ablation_results.{md,json}` (nothing typed by hand).
Webots numbers: **none yet**.

| ID | Ablation | Offline (kinematic) | Webots scenario (`tools/run_scenario.py`) | Metrics |
|---|---|---|---|---|
| N1 | odometry-only vs GPS-only vs EKF | `run_mission(localization=odom/gps/ekf)`, GPS σ 5 cm, IMU σ 1°, 2 % encoder scale error, 3 seeds + one 10 s GPS dropout | `A_odom_only_P3_B`, `A_gps_only_P3_B` vs `S1_store_P3_B` | pose RMSE / max vs ground truth, mean NEES, mission result (`mission_report.json → localization_vs_ground_truth`) |
| N2 | fixed speed (0.30 m/s) vs speed governor (0.45 m/s cruise) | 3 missions | `A_fixed_speed_P3_B` vs `S1_store_P3_B` | mission + navigation time, min clearance, contacts, emergency stops |
| N3 | safety-stop-only replanning vs continuous dynamic layer | static box on the route, crossing cart × 3 phases | `A_static_replan_P3_B` vs `S1_store_P3_B` (carts `CART_1/2`, `FAST_*` in the world) | result, time, contacts, min clearance, replans by reason |
| N4 | displacement mid-route: EKF without vs with relocalisation | 3 seeds | manual: drag the robot in DELIVER (README) | pose RMSE, relocalisations, replans, result |

Offline results (2026-11-10, 107 s run) — summary; full tables in
`docs/evidence/experiments/ablation_results.md`:

* N1: EKF RMSE 0.009–0.012 m, 4/4 DONE (incl. the GPS dropout); GPS-only 0.070 m,
  0/4 DONE (docking SETTLE never sees a still pose — raw GPS jitter); odometry-only
  0.22–0.26 m, 2/4 DONE. Mean NEES (EKF) 1.1–1.6.
* N2: navigation time 26.6→22.6 s, 29.1→24.8 s, 28.5→24.5 s (−14…−15 %); mission
  time −2…−5 %; min clearance unchanged; 0 contacts; 6/6 parcels in slot.
* N3: static box — 96.4 → 85.3 s, min clearance 0.104 → 0.178 m, early detour
  instead of an 8 s stop. Crossing cart — contacts 3 → 1 over three phases; the
  full stack waits at the crossing (phase 0: 92.0 → 104.7 s).
* N4: relocalisation 3/3 DONE, RMSE 0.045–0.046 m; without it 3/3 DONE in the
  kinematic model but RMSE 0.13–0.18 m and up to +40 s.

Rules from 18.5 apply: offline and Webots results are never mixed in one table;
report k/N.
