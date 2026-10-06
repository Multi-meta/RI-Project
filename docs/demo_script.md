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

## Question triage

- Kinematics/control → person A (kinematics.py, waypoint_follower.py)
- Perception/vision → person B (perception.py, detector.py, estimation.py)
- Planning/FSM/integration → person C (mapping.py, a_star.py, decision_maker.py)
- Anything unmeasured: say so honestly and point to the tool that measures it.
