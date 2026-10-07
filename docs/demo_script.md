# Demo Script — Mid-Evaluation (5 min talk + 2 min Q&A)

## Before the demo (day before)

1. `pip install -r requirements.txt` and set Webots Preferences → Python
   command to that interpreter.
2. `python tools/generate_world.py` (fresh world) and `python -m pytest -q`
   (expect 63 passed).
3. Record a full run as backup video (≤60 s) — if the live run fails, play
   the recording. Save to `docs/evidence/run_full.mp4`.
4. Have open in tabs: `docs/evidence/astar_pickup_to_zoneB.png`,
   `docs/evidence/metrics.md`, console window.

## Live demo flow (fallback: play recording)

| Step | Action | What to say |
|---|---|---|
| 0 | Open `worlds/warehouse.wbt` | "World is GENERATED from config.py — the planner's map and the simulator can never disagree." |
| 1 | Press ▶, point at console | "FSM starts: IDLE → FIND_PACKAGE. The robot rotates and the OpenCV detector looks for the blue package." |
| 2 | `Package P3 detected at (x, y)` appears | "Detection + LiDAR range → world position estimate through our transformation chain." |
| 3 | `A* path: N waypoints...` | "A* on the inflated occupancy grid — no corner cutting, octile heuristic, verified optimal against Dijkstra." |
| 4 | Watch the robot follow | "PID heading control with speed shaping; LiDAR safety stop watches ±30° ahead." |
| 5 | `PICK placeholder` | "Mid-eval placeholder; final eval attaches the package via connector/supervisor." |
| 6 | `MISSION REPORT` | "Time, path length, replans, collisions — all logged to CSV." |

## Recording the backup video (checklist 8.4)

1. Open `worlds/warehouse.wbt`, let the run reach DONE once (warm-up).
2. Webots: **Tools → Movie → Record Movie** (or the camera icon) → choose
   MP4, quality ~80 %, and restart the simulation — it records until you stop.
3. Stop right after `MISSION REPORT` prints (≤60 s), trim if needed, save as
   `docs/evidence/run_full.mp4`.
4. Also capture the console output to `docs/evidence/console_log.txt`
   (right-click console → Copy all).
5. Fallback if Webots movie capture is flaky: record the screen with
   Win+G (Xbox Game Bar) while the sim runs in fast-forward
   (`--mode=fast`).

## Rehearsal notes (checklist 8.5)

Every member must be able to explain EVERY slide; the primary presenter per
slide is:

| Slide | Content | Primary | Backup |
|---|---|---|---|
| 1 | Title / status chips | A | C |
| 2 | Mission + 7 capabilities | C | A |
| 3 | Architecture | C | B |
| 4 | Build status (modules, tests) | A | C |
| 5 | Generated world | A | C |
| 6 | A* planning validated | C | A |
| 7 | Perception & control laws | B | A |
| 8 | Verified vs measured vs pending | B | C |
| 9 | Team & roadmap | C | B |
| 10 | Closing | C | A |

Rehearsal checklist (do it once without notes):
- A can derive v = r(ωR+ωL)/2, ω = r(ωR−ωL)/L and explain the wheel-sphere
  bounding-object fix (-14 % yaw rate) and the caster zero-friction material.
- B can explain the two-light HSV fix (dark faces → V ≈ 46 < threshold) and
  the pixel→world chain on the whiteboard.
- C can explain octile admissibility, no-corner-cutting, and walk the FSM
  states with the run.log timeline.
- All numbers said aloud must exist in `docs/evidence/` — if you can't point
  to the file, don't say the number.

## Question triage

- Kinematics/control → person A (kinematics.py, waypoint_follower.py)
- Perception/vision → person B (perception.py, detector.py, estimation.py)
- Planning/FSM/integration → person C (mapping.py, a_star.py, decision_maker.py)
- Anything unmeasured: say so honestly and point to the tool that measures it.
