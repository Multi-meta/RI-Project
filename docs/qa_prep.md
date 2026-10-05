# Viva / Q&A Prep

Short, correct answers to likely evaluator questions. Owners: A (sim &
control), B (perception), C (planning & decisions).

## Kinematics & locomotion (A)

1. **Derive the differential-drive forward kinematics.**
   Contact-point velocities: v = r(ωR+ωL)/2 (average of wheel linear speeds);
   rotation rate ω = r(ωR−ωL)/L (difference over separation L). Positive ω =
   CCW (left turn) because the right wheel must go faster.

2. **What does inverse kinematics do here?**
   Maps a commanded body twist (v, ω) to wheel speeds:
   ωR = (v + ωL/2)/r, ωL = (v − ωL/2)/r, where the second L is the
   wheel separation.

3. **Why scale BOTH wheels when clamping?**
   Scaling both by the same factor preserves the v/ω ratio — i.e. the same
   turning curvature — just slower. Scaling only one would change the path.

4. **How does odometry work? What are its error sources?**
   Arc model per step: Δs = r(ΔφR+ΔφL)/2, Δθ = r(ΔφR−ΔφL)/L, integrate x,y
   with the mid-step heading. Errors: wheel slip, unequal wheel diameter,
   encoder quantization — drift grows unbounded, hence validation vs GPS.

5. **Why ENU and FLU?**
   ENU = world axes (x east, y north, z up); FLU = robot body axes
   (x forward, y left, z up). Consistent handedness keeps yaw CCW-positive
   and transform signs (x_w = x_r + d·cos(θ+φ)) unambiguous.

## Sensors & perception (B)

6. **Why LiDAR + camera together?**
   Camera gives semantics (which package by color); LiDAR gives metric range
   (depth + safety stop). Neither alone suffices: color has no reliable
   depth, range has no identity.

7. **How does HSV detection work? Why HSV over RGB?**
   Hue encodes color separately from value (brightness) and saturation, so
   moderate lighting changes don't break thresholds. We threshold hue/sat/val
   windows per color, then morphological open/close clean the mask.

8. **Why does red need two hue ranges?**
   OpenCV hue is 0–180; red wraps around 0, so red pixels sit near both 0–10
   AND 170–180.

9. **How can zone colors never be confused with packages?**
   Zones are yellow (hue ≈ 30) and cyan (≈ 90); package windows are green
   40–80 and blue 105–135, deliberately disjoint. Tested in pytest.

10. **Explain the pixel→world chain.**
    Pixel column u → bearing φ = −atan2(u − W/2, f_px) with f_px = (W/2)/tan(FOV/2)
    (minus because image x grows right, robot bearings grow left); depth d
    from the LiDAR minimum in a ±3° window at φ (fallback: pinhole
    d = f_px·PKG_SIZE/bbox_width); world position = pose + d·(cos(θ+φ),
    sin(θ+φ)). This is the frames/transformation exercise of the course.

11. **Is the detection score a probability?**
    No — a documented heuristic: contour solidity × size factor. We call it
    "heuristic score" on the slides.

12. **What does the LiDAR safety stop do?**
    If any ray within ±30° of forward is < 0.30 m, command v = 0 and set
    status BLOCKED. It's a local reactive layer under the planner.

13. **Why must shadows be disabled in the world?**
    Shadows/specular highlights shift V (and apparent hue saturation), moving
    pixels out of the HSV windows — flat lighting keeps masks stable.

## Mapping & planning (C)

14. **Why inflate obstacles?**
    Planning treats the robot as a point; growing obstacles by robot radius
    (0.18 m) + safety margin (0.07 m) makes any point-path collision-free for
    the real body. Circular kernel: a square kernel would close diagonal gaps
    the robot can physically pass.

15. **Why A* and why the octile heuristic?**
    A* = Dijkstra + an admissible heuristic that focuses the search. Octile
    distance = dx+dy+(√2−2)·min(dx,dy) is the EXACT path cost on an
    8-connected grid, so it never overestimates → optimality preserved. We
    verified A* cost == Dijkstra cost on 60 random grids.

16. **What is corner cutting and why forbid it?**
    A diagonal move between two cells that are diagonally adjacent to an
    obstacle would clip the obstacle corner with the robot body. We allow a
    diagonal only when BOTH adjacent orthogonal cells are free.

17. **Occupancy grid conventions?**
    Cell (row, col); col = floor((x−x_min)/res), row = floor((y−y_min)/res);
    cell_to_world returns the CENTER; res = 0.1 m; static layer from config,
    dynamic layer from LiDAR (final eval); planning uses static|dynamic
    inflated.

18. **Global planning vs local avoidance?**
    A* is global (known map, full route); the LiDAR safety stop is local
    (reactive, current scan). They complement: planner for optimality,
    reflex layer for safety.

19. **Path simplification?**
    Bresenham line-of-sight: keep a waypoint only when the straight grid line
    to it is collision-free — turns 31 cells into 2 waypoints, then resample
    at 0.3 m for smooth following.

## Control (A)

20. **How does the waypoint follower work?**
    Per step: heading error = wrap(atan2(dy,dx) − θ); ω = PID(error); v =
    V_MAX·max(0, cos(error))·min(1, dist/SLOWDOWN). If |error| > 0.6 rad,
    rotate in place first. Waypoint reached < 0.10 m, goal < 0.15 m.

21. **What does each PID gain do?**
    kp: proportional steering strength (too low = lazy, too high = oscillation).
    ki: removes steady-state offset (e.g. asymmetric wheel friction); with
    anti-windup. kd: damps overshoot by reacting to error rate.

22. **What is anti-windup and why needed?**
    While the output is saturated, a large error keeps integrating and the
    actuator can't respond → huge overshoot later. We freeze the integrator
    when saturated and the error pushes further (conditional integration).

23. **What is cross-track error?**
    Perpendicular distance from the robot to the current path segment — the
    following-accuracy metric (mean/max reported per run).

## Architecture & decisions (C)

24. **What are the FSM states?**
    IDLE → FIND_PACKAGE (rotate + detect) → NAVIGATE (A* to approach pose) →
    PICK (placeholder) → PLAN_DELIVERY (A* to zone) → DELIVER → DONE; ERROR
    on timeout/failed plan. Every transition printed with a timestamp.

25. **How is this different from a scripted route?**
    Nothing is hard-coded: the package position comes from the camera+LiDAR
    estimate, the route from A* on the occupancy grid, the motion from closed-
    loop control. Change the layout in config and the robot re-plans without
    code changes.

26. **Why GPS+IMU for pose instead of odometry?**
    Mid-eval reliability choice: GPS+IMU doesn't drift. We IMPLEMENT odometry
    in parallel and quantify its drift against GPS (deliverable). Honest
    limitation: GPS is idealized; a real robot needs sensor fusion / SLAM.

27. **What would change on a real robot?**
    Noisy/absent GPS → LiDAR localization/SLAM; detection must survive real
    lighting (our dim/bright experiments); motor models with backlash; real
    gripper; safety certification.

28. **How is the world kept consistent with the map?**
    Generated from the same config.py; a supervisor check at startup compares
    every DEF position against config (warns at >1 cm).

29. **What's ready for the final evaluation?**
    Hooks: add_obstacle_world/clear_dynamic/path_is_blocked (tested),
    follower BLOCKED status + FSM replan slot, supervisor access for spawning
    obstacles, DYN_OBSTACLE DEF pre-placed, PICK_DISTANCE_THRESHOLD config,
    mission report prints replans/collisions.
