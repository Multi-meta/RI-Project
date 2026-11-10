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
