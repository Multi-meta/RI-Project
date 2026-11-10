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

## Robotic arm, gripper, shelving and cameras (final evaluation — Ankit)

30. **Describe the arm.**
    A 3-joint planar arm mounted on a post at the back of the base: shoulder,
    elbow and wrist are revolute joints about the robot's y axis, plus a
    parallel gripper with two sliding fingers. The base gives the yaw and the
    gross position; the arm gives reach (x) and height (z); the wrist keeps
    the gripper horizontal (φ = q1+q2+q3 = 0). Each joint is a Webots
    HingeJoint + RotationalMotor + PositionSensor; each finger a SliderJoint +
    LinearMotor + PositionSensor with a force TouchSensor pad.

31. **Derive the inverse kinematics.**
    Wrist point W = TCP − Lt(cos φ, sin φ). With d = |W − S|:
    cos q2 = (d² − L1² − L2²)/(2 L1 L2) (law of cosines; |cos q2| > 1 →
    unreachable). Elbow-up: q2 = −acos(·). q1 = atan2(Wz−Sz, Wx−Sx) −
    atan2(L2 sin q2, L1 + L2 cos q2). q3 = φ − q1 − q2. Tests check IK→FK
    round trip to 1e-9 m.

32. **Why elbow-up?** It keeps the elbow above the parcel and the camera; the
    elbow-down solution of the carry pose violates the wrist limit, and for
    shelf poses it would swing the forearm through the robot body.

33. **Why is the arm at the back of the robot?** Anything attached to the top
    of the body crosses the LiDAR plane (0.14 m). At the back the only
    self-return is a ~13° sector straight behind (removed by a 0.15 m
    self-filter); the front LiDAR sector, the front camera and the front bumper
    stay completely free.

34. **Webots LiDARs ignore the robot, don't they?** No — the manual says
    objects closer than minRange "are not detected (but still occlude other
    objects)". So the post would occlude; we filter its returns and accept the
    small rear blind sector.

35. **How did you choose link lengths and the mount?** With a 2-D clearance
    model (arm links, palm, fingers and the carried parcel vs body, bumpers,
    camera housings, post, floor and rack) evaluated along every planned
    motion. The first design put the forearm through the carried parcel and
    the wrist into the bumper; the search found shoulder height 0.25 m and
    links 0.28/0.25 m with ≥ 1 cm clearance everywhere and ≤ 82 % of full
    reach (away from the singular stretched pose).

36. **How are joint motions generated?** Joint moves: quintic polynomial
    (zero velocity/acceleration at both ends) timed from the joint velocity
    limits. Straight TCP lines: IK every 1 cm, bisected until joints change
    < 0.05 rad between samples, then timed by the slowest joint per segment so
    no joint exceeds 80 % of its velocity limit (tested).

37. **How do you know the arm is blocked?** Position sensors: if
    |q_cmd − q_measured| > 0.25 rad for 0.4 s the motion FAULTS and the arm
    holds. During INSERT that is reported as SLOT_BLOCKED and the FSM tries the
    next free slot with the parcel still latched.

38. **How does the gripper hold the parcel?** Fingers close force-limited
    (12 N available) and stall on the parcel sides; then a Webots Connector
    latch (active on the palm, passive on each parcel face) is locked, which
    creates a real physical joint. Release = unlock + open.

39. **Isn't the Connector a cheat?** It is the documented Webots mechanism for
    gripping objects with a passive connector, i.e. a model of a mechanical
    latch/magnet. It only locks when the peer is geometrically present
    (≤ 4 cm, axes opposed within 0.35 rad), `snap` is off so the parcel is not
    moved, and the holding decision never relies on the latch alone. No
    Supervisor teleport is used with the arm.

40. **How is the attachment verified?** Sensor fusion: finger width =
    parcel size ± 1.5 cm (encoders) AND palm range-finder ≤ 3 cm AND (both pad
    forces ≥ 2 N OR latch locked). Checked before latching, after a 5 cm lift
    and again in the carry pose. A missed grasp shows as width ≈ 2 cm; a
    dropped parcel as palm range ≫ 3 cm.

41. **How is the delivery verified?** After release the gripper backs off
    6 cm; the palm range-finder must read 7 ± 2 cm (the parcel stayed in the
    slot), the latch must be open and the fingers open. Only then does the FSM
    reach DONE. Separately, the Supervisor audits the parcel's true position
    for the report; if the two disagree the console warns.

42. **What is docking and why is it needed?** A* + the follower stop within
    ~0.1–0.4 m; the gripper tolerates only ±3 cm laterally. Docking: face the
    target, fit the parcel's LiDAR outline (centre + yaw, corner detection) →
    dock line; point-stabilise to a pre-dock point; face; refit from the
    square-on view; creep in with the LiDAR range as feedback to ±0.8 cm;
    settle. Final lateral error must be ≤ 1.5 cm, heading ≤ 3°.

43. **What went wrong first in docking?** Seen obliquely, the LiDAR saw two
    faces of the parcel; a single line fit gave a diagonal heading and the
    palm check failed (correctly — no false success). Fix: corner detection,
    centre from the corner, choose the face that looks at the aisle, refine
    twice.

44. **Why these shelf heights?** L1 = 0.15 m (its kick plate is in the LiDAR
    plane → exact docking range), L2 = 0.41 m (gap leaves 2.5 cm above a
    parcel on L1 while inserting), L3 = 0.67 m is beyond reach (wrist would
    need 0.59 m from the shoulder > L1 + L2 = 0.53 m) — overflow storage.

45. **How does the planner know about the racks?** Their footprints are in
    `STATIC_OBSTACLES`, the same list the world generator uses; tests check
    the pre-dock poses are free, reachable, and that two routes still exist.

46. **Will the robot tip over?** Static check: with a 0.5 kg parcel at full
    reach the centre of mass is 4 cm ahead of the axle, 7 cm behind the front
    caster, thanks to a 0.8 kg counterweight modelled in the base's COM.

47. **What was wrong with the cameras?** The planar FOV was set to π (Webots
    allows ≤ π; at π the focal length is ~0 so the image collapses); config
    and the hand-edited world disagreed; the rear camera's housing was in
    front of its lens; mount offsets were hard-coded. Fixed by a valid 1.0 rad
    planar FOV, regenerating from config, housings behind the lenses.

48. **How do you convert Webots images?** `getImage()` is BGRA, row-major,
    top-left first — OpenCV's own layout — so we only drop alpha: no RGB swap,
    no flip. Tests check channel order and orientation pixel by pixel.

49. **Why is the rear image not mirrored, and how do you get bearings?** The
    rear camera is the same camera rotated π about z — it sees what you would
    see standing behind the robot looking back. Image-right is robot-left, so
    φ = π − atan((u − cx)/f); tests project 3-D points through both cameras
    and recover the true bearing.

50. **How can front and rear never be swapped?** Each physical camera is a
    `CameraChannel` bound to its role by device name; the role travels with
    every frame and detection and decides the bearing formula. The Webots
    check compares each camera's detections with that camera's own
    Recognition node.

51. **What does "R scaled by light level" mean (11.5)?** Camera bearing
    variance R = R₀·(nominal brightness / mean)² and ∞ below a dark threshold,
    so the Kalman gain of the camera shrinks smoothly as lights dim and the
    update is skipped in the dark — de-weighting without a hard switch.

52. **What is still unproven?** Everything physical: grip stability under
    ODE contacts, real docking accuracy, success rate over ≥ 10 Webots runs,
    camera evidence images. The offline kinematic simulator proves the logic
    and geometry, not the physics.


## EKF navigation, dynamic replanning, safety (Ankit, fix/ankit-ekf-navigation-manipulation)

1. **Why did the robot replan from its old position after being pushed?**
   The wheels did not see the push, so the EKF prediction stayed at the old
   place; every later GPS fix failed the χ² gate and was rejected forever.
   Now 0.5 s of mutually consistent rejected fixes ⇒ relocalise on GPS+IMU and
   trigger a replan from the new pose. Single outliers are still rejected.

2. **What was wrong with the EKF prediction?** `ds = hypot(Δx, Δy)` is never
   negative (reversing predicted forward motion) and Δθ was not wrapped. Now the
   signed arc length and Δθ come straight from the encoder increments.

3. **How do you tell static from dynamic obstacles?** A LiDAR hit on a cell of
   the known map (± one cell for pose error) is the static map; other hits add
   evidence in the dynamic layer. A cell that was occupied and is then seen
   free was vacated ⇒ the obstacle moves ⇒ yield instead of detouring.

4. **How do you avoid ghost obstacles but still remember a blocked route?**
   Only *observed-free* cells are cleared (a ray passes through them);
   unobserved cells keep their evidence until a 30 s TTL.

5. **Stopping distance?** `d = v²/(2a) + v·t_react + margin`
   (0.8 m/s², 0.1 s, 0.1 m ⇒ 0.27 m at 0.45 m/s). The governor inverts it:
   the speed is the largest one whose stopping distance fits the measured
   clearance in the swept corridor.

6. **Why scale v and w together?** Keeps the curvature of the follower's arc,
   so slowing down never changes the path.

7. **When may the robot reverse in recovery?** Only if the rear LiDAR sector
   has valid (non-NaN) data and its corridor clearance exceeds the backup
   distance + margin. Missing data ≠ free.

8. **What did you do about the arm freeze without logs?** Removed every way
   the sequence can hang or crash: window-based gripper stall test, settle under
   load only when stopped, motion-based step deadlines, exception → FAILED
   (and a controller-level guard), start-up encoder wait; plus a per-joint
   diagnosis (target/measured/torque) on every fault for the next occurrence.
