"""Speed governor, dynamic obstacle layer, safe-start planning and replanning.

Pure-logic tests (no Webots). The FSM-level tests drive DecisionMaker with
a scripted unicycle and synthetic LiDAR scans built from boxes.
"""

import math

import numpy as np
import pytest

import config
import path_utils
from decision_maker import DecisionMaker
from dynamic_map import DynamicObstacleMap
from mapping import OccupancyGrid
from safety import (SpeedGovernor, clearance_speed, corridor_clearance,
                    stopping_distance)

N_RAYS = 360
ANGLES = np.array([-math.pi + i * 2 * math.pi / N_RAYS for i in range(N_RAYS)])


def scan_boxes(pose, boxes, rmax=4.0):
    """Ray-cast (x0, x1, y0, y1) boxes from pose; inf where nothing is hit."""
    x, y, th = pose
    out = np.full(N_RAYS, np.inf)
    for i, a in enumerate(ANGLES):
        dx, dy = math.cos(th + a), math.sin(th + a)
        best = np.inf
        for (x0, x1, y0, y1) in boxes:
            tl, th_ = -np.inf, np.inf
            for o, d, lo, hi in ((x, dx, x0, x1), (y, dy, y0, y1)):
                if abs(d) < 1e-12:
                    if not lo <= o <= hi:
                        tl, th_ = np.inf, -np.inf
                    continue
                t1, t2 = (lo - o) / d, (hi - o) / d
                tl, th_ = max(tl, min(t1, t2)), min(th_, max(t1, t2))
            if th_ >= max(tl, 0) and tl > 0:
                best = min(best, tl)
        out[i] = best if best <= rmax else np.inf
    return out


def empty_grid():
    g = OccupancyGrid(-3, 3, -2, 2, 0.1)
    g.add_static_rect(0, 2.0, 6.0, 0.1)
    g.add_static_rect(0, -2.0, 6.0, 0.1)
    return g


# ------------------------------------------------------------------ safety
def test_stopping_distance_formula():
    v = 0.4
    a, t, m = config.SAFETY_BRAKE_DECEL, config.SAFETY_REACTION_TIME, config.SAFETY_STOP_MARGIN
    assert stopping_distance(v) == pytest.approx(v * v / (2 * a) + v * t + m)
    # clearance_speed is the inverse of stopping_distance
    for c in (0.15, 0.3, 0.6, 1.2):
        assert stopping_distance(clearance_speed(c)) == pytest.approx(c, abs=1e-9)
    assert clearance_speed(config.SAFETY_STOP_MARGIN) == 0.0


def test_corridor_clearance_ignores_side_obstacles_and_respects_direction():
    pose = (0.0, 0.0, 0.0)
    ahead = scan_boxes(pose, [(1.0, 1.2, -0.05, 0.05)])
    side = scan_boxes(pose, [(0.3, 0.5, 0.40, 0.60)])
    behind = scan_boxes(pose, [(-1.2, -1.0, -0.05, 0.05)])
    assert corridor_clearance(ahead, ANGLES) == pytest.approx(1.0 - config.ROBOT_FRONT_EXTENT, abs=0.02)
    assert corridor_clearance(side, ANGLES) == math.inf
    assert corridor_clearance(behind, ANGLES) == math.inf
    assert corridor_clearance(behind, ANGLES, reverse=True) < 1.0


def test_governor_caps_speed_by_clearance_and_preserves_curvature():
    gov = SpeedGovernor(enabled=True)
    scan = scan_boxes((0, 0, 0), [(0.4, 0.6, -0.1, 0.1)])
    clr = corridor_clearance(scan, ANGLES)
    v = w = None
    for _ in range(200):                      # let the ramp settle
        v, w = gov.limit(0.45, 0.9, 0.032, scan, ANGLES)
    assert v == pytest.approx(clearance_speed(clr), rel=1e-3)
    assert w / v == pytest.approx(0.9 / 0.45, rel=1e-3)   # same arc
    assert gov.last["reason"] == "clearance"
    assert stopping_distance(v) <= clr + 1e-9


def test_governor_acceleration_and_emergency():
    gov = SpeedGovernor(enabled=True)
    v, _ = gov.limit(0.45, 0.0, 0.032, None, None)
    assert v == pytest.approx(config.SAFETY_ACCEL * 0.032)
    for _ in range(100):
        v, _ = gov.limit(0.45, 0.0, 0.032, None, None)
    assert v == pytest.approx(config.V_CRUISE)
    close = scan_boxes((0, 0, 0), [(config.ROBOT_FRONT_EXTENT + 0.05, 0.5, -0.1, 0.1)])
    v, w = gov.limit(0.45, 0.3, 0.032, close, ANGLES)
    assert v == 0.0 and w == 0.0 and gov.emergency_stops == 1


def test_governor_health_caps():
    gov = SpeedGovernor(enabled=True)
    for _ in range(100):
        v, _ = gov.limit(0.45, 0.0, 0.032, None, None, health={"localization": "DEAD_RECKONING"})
    assert v == pytest.approx(config.V_DEGRADED)
    nan_scan = np.full(N_RAYS, np.nan)
    for _ in range(100):
        v, w = gov.limit(0.45, 0.5, 0.032, nan_scan, ANGLES)
    assert v == 0.0 and w == 0.0
    gov2 = SpeedGovernor(enabled=True)
    for _ in range(100):
        v, _ = gov2.limit(0.45, 0.0, 0.032, None, None, carrying=True)
    assert v == pytest.approx(config.V_CARRY)


def test_governor_disabled_is_transparent():
    gov = SpeedGovernor(enabled=False)
    assert gov.limit(0.3, 0.2, 0.032, None, None) == (0.3, 0.2)


def test_dock_mode_skips_clearance_limit():
    gov = SpeedGovernor(enabled=True)
    close = scan_boxes((0, 0, 0), [(0.25, 0.5, -0.1, 0.1)])
    v, _ = gov.limit(0.05, 0.0, 0.5, close, ANGLES, mode="DOCK")
    assert v == pytest.approx(0.05)


# ------------------------------------------------------------- dynamic map
def test_dynamic_map_marks_new_obstacle_not_static_map():
    g = empty_grid()
    m = DynamicObstacleMap(g)
    pose = (0.0, 0.0, 0.0)
    box = (1.0, 1.3, -0.15, 0.15)
    wall = (-3, 3, 1.95, 2.05)
    for k in range(3):
        m.update(pose, scan_boxes(pose, [box, wall]), ANGLES, k * 0.2)
    assert g.dynamic[g.world_to_cell(1.05, 0.0)]
    r, c = g.world_to_cell(0.5, 1.99)
    assert not g.dynamic[r, c]                      # wall = static map, not dynamic


def test_single_return_does_not_block_and_free_rays_clear():
    g = empty_grid()
    m = DynamicObstacleMap(g)
    pose = (0.0, 0.0, 0.0)
    box = (1.0, 1.3, -0.15, 0.15)
    m.update(pose, scan_boxes(pose, [box]), ANGLES, 0.0)
    assert m.count() == 0                           # one hit < DYN_OCCUPIED_LEVEL
    m.update(pose, scan_boxes(pose, [box]), ANGLES, 0.2)
    assert m.count() > 0
    for k in range(6):                              # the box moved away
        m.update(pose, scan_boxes(pose, []), ANGLES, 0.4 + 0.2 * k)
    assert m.count() == 0                           # no ghost obstacle
    assert m.moving_near(1.05, 0.0, 0.3, 1.4)       # ... and it was a MOVER


def test_unobserved_obstacle_is_remembered_then_expires():
    g = empty_grid()
    m = DynamicObstacleMap(g)
    box = (1.0, 1.3, -0.15, 0.15)
    for k in range(3):
        m.update((0, 0, 0), scan_boxes((0, 0, 0), [box]), ANGLES, 0.2 * k)
    # robot now looks elsewhere: rays never pass through the box cells
    far = (-2.0, 1.0, math.pi)
    narrow = np.where(np.abs(ANGLES) < 0.3, np.inf, np.nan)
    m.update(far, narrow, ANGLES, 1.0)
    assert m.count() > 0                            # memory of the blocked route
    m.update(far, narrow, ANGLES, 1.0 + config.DYN_TTL + 1)
    assert m.count() == 0                           # expired


def test_keep_out_and_own_footprint_never_marked():
    g = empty_grid()
    m = DynamicObstacleMap(g)
    box = (0.5, 0.7, -0.1, 0.1)
    for k in range(3):
        m.update((0, 0, 0), scan_boxes((0, 0, 0), [box]), ANGLES, 0.2 * k,
                 keep_out=[(0.6, 0.0, 0.3)])
    assert m.count() == 0


def test_untrusted_pose_skips_mapping():
    g = empty_grid()
    m = DynamicObstacleMap(g)
    for k in range(3):
        m.update((0, 0, 0), scan_boxes((0, 0, 0), [(1, 1.2, -0.1, 0.1)]), ANGLES,
                 0.2 * k, trusted=False)
    assert m.count() == 0


def test_external_marks_are_pinned():
    g = empty_grid()
    m = DynamicObstacleMap(g)
    g.add_obstacle_world(-1.0, 1.0, 0.1)            # e.g. a non-target parcel
    for k in range(5):
        m.update((0, 0, 0), scan_boxes((0, 0, 0), []), ANGLES, 0.2 * k)
    assert g.dynamic[g.world_to_cell(-1.0, 1.0)]


# ------------------------------------------------------------ safe start
def test_safe_start_moves_out_of_inflation_without_crossing_obstacle():
    g = empty_grid()
    g.add_obstacle_world(0.35, 0.0, 0.15)           # obstacle just ahead
    occ = g.inflate(config.INFLATION_RADIUS)
    assert occ[g.world_to_cell(0.0, 0.0)]           # robot cell inside the inflation
    wps, info = path_utils.plan_to(g, (0.0, 0.0), (2.0, 0.0), inflation_radius=config.INFLATION_RADIUS,
                                   safe_start_radius=config.SAFE_START_RADIUS)
    assert wps is not None
    assert 0 < info["start_shift_m"] <= config.SAFE_START_RADIUS
    sx, sy = wps[0]
    assert sx < 0.25                                # did not jump through/past the obstacle
    assert math.hypot(sx, sy) <= config.SAFE_START_RADIUS + 0.08


def test_safe_start_refuses_when_boxed_in():
    g = empty_grid()
    for ang in np.linspace(0, 2 * math.pi, 16, endpoint=False):
        g.add_obstacle_world(0.3 * math.cos(ang), 0.3 * math.sin(ang), 0.08)
    wps, info = path_utils.plan_to(g, (0.0, 0.0), (2.0, 0.0), inflation_radius=config.INFLATION_RADIUS,
                                   safe_start_radius=config.SAFE_START_RADIUS)
    assert wps is None and info["reason"] == "START_BLOCKED"
    assert g.dynamic[g.world_to_cell(0.3, 0.0)]     # obstacles were NOT cleared


# ---------------------------------------------------- FSM-level replanning
class Unicycle:
    def __init__(self, pose):
        self.x, self.y, self.th = pose

    def pose(self):
        return (self.x, self.y, self.th)

    def step(self, v, w, dt):
        self.th = math.atan2(math.sin(self.th + w * dt), math.cos(self.th + w * dt))
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt


def deliver_fsm(grid, start=(-2.0, 0.0, 0.0), goal=(2.5, 0.0), governor=True):
    def planner(s, g):
        return path_utils.plan_to(grid, s, g, resample_spacing=0.3,
                                  inflation_radius=config.INFLATION_RADIUS,
                                  safe_start_radius=config.SAFE_START_RADIUS)
    logs = []
    fsm = DecisionMaker(planner, lambda *a: None, grid=grid, log=logs.append,
                        obstacle_map=DynamicObstacleMap(grid),
                        governor=SpeedGovernor(enabled=governor))
    config.ZONE_OF["T"] = goal
    old_zone = config.TASK_ZONE
    config.TASK_ZONE = "T"
    fsm.state = "PLAN_DELIVERY"
    fsm.carrying = True
    return fsm, logs, old_zone


def drive(fsm, sim, boxes_fn, t0=0.0, T=40.0, dt=0.032, until=("DONE", "ERROR")):
    t = t0
    min_clear = math.inf
    while t < t0 + T and fsm.state not in until:
        scan = scan_boxes(sim.pose(), boxes_fn(t))
        v, w = fsm.update(sim.pose(), scan, ANGLES, [], t, dt)
        sim.step(v, w, dt)
        min_clear = min(min_clear, corridor_clearance(scan, ANGLES))
        t += dt
    return t, min_clear


def test_static_obstacle_on_route_replans_before_the_safety_stop():
    grid = empty_grid()
    fsm, logs, old = deliver_fsm(grid)
    try:
        sim = Unicycle((-2.0, 0.0, 0.0))
        box = (0.0, 0.3, -0.25, 0.25)
        t, _ = drive(fsm, sim, lambda t: [box], T=60.0)
        assert fsm.state == "DONE"
        assert fsm.replan_reasons.get("PATH_BLOCKED", 0) >= 1
        assert "SAFETY_STOP" not in fsm.replan_reasons      # detoured early
        ev = fsm.replan_events[0]
        assert ev["start"][0] < -0.4                        # replanned from where it WAS
    finally:
        config.TASK_ZONE = old
        config.ZONE_OF.pop("T", None)


def test_replan_starts_from_current_pose_after_displacement():
    grid = empty_grid()
    fsm, logs, old = deliver_fsm(grid)
    try:
        sim = Unicycle((-2.0, 0.0, 0.0))
        drive(fsm, sim, lambda t: [], T=2.0, until=("DONE", "ERROR"))
        sim.x, sim.y = sim.x + 0.2, sim.y + 1.0              # pushed sideways
        fsm.notify_pose_jump((0.2, 1.0, 0.0), 2.0)
        scan = scan_boxes(sim.pose(), [])
        fsm.update(sim.pose(), scan, ANGLES, [], 2.05, 0.032)
        ev = fsm.replan_events[-1]
        assert ev["reason"] == "POSE_JUMP"
        assert ev["start"] == pytest.approx((round(sim.x, 3), round(sim.y, 3)), abs=1e-3)
        assert fsm.follower.idx == 0                         # fresh follower
        drive(fsm, sim, lambda t: [], t0=2.1, T=60.0)
        assert fsm.state == "DONE"
    finally:
        config.TASK_ZONE = old
        config.ZONE_OF.pop("T", None)


def test_moving_obstacle_is_yielded_to_not_chased():
    grid = empty_grid()
    fsm, logs, old = deliver_fsm(grid)
    try:
        sim = Unicycle((-2.0, 0.0, 0.0))

        def cart(t):                       # crosses the route at x = 0.5
            y = -1.5 + 0.5 * t if t < 6.0 else 1.5
            return [(0.4, 0.7, y - 0.2, y + 0.2)]
        t, min_clear = drive(fsm, sim, cart, T=60.0)
        assert fsm.state == "DONE"
        assert any("MOVING obstacle" in m for m in logs)
        assert min_clear > config.SAFETY_STOP_MARGIN - 0.03
    finally:
        config.TASK_ZONE = old
        config.ZONE_OF.pop("T", None)


def test_no_path_runs_bounded_recovery_then_error():
    grid = empty_grid()
    grid.add_static_rect(1.0, 0.0, 0.2, 3.9)                # wall across the arena
    fsm, logs, old = deliver_fsm(grid)
    try:
        sim = Unicycle((-2.0, 0.0, 0.0))
        fsm.update(sim.pose(), scan_boxes(sim.pose(), []), ANGLES, [], 0.0, 0.032)
        assert fsm.state == "ERROR"                          # no path at all: honest error
    finally:
        config.TASK_ZONE = old
        config.ZONE_OF.pop("T", None)


def test_recovery_never_reverses_without_verified_rear():
    fsm = DecisionMaker(lambda s, g: (None, {"reason": "NO_PATH"}), lambda *a: None,
                        grid=empty_grid(), log=lambda m: None,
                        obstacle_map=DynamicObstacleMap(empty_grid()))
    fsm.state = "DELIVER"
    from waypoint_follower import WaypointFollower
    fsm.follower = WaypointFollower([(1.0, 0.0)])
    fsm._start_recovery(0.0, "REPLAN_FAILED/TEST")
    nan_rear = np.where(np.abs(ANGLES) > 2.0, np.nan, 0.5)   # rear data invalid
    cmds = []
    t = 0.0
    while t < 150.0 and fsm.state == "DELIVER":
        cmds.append(fsm._recovery_step((0.0, 0.0, 0.0), nan_rear, ANGLES, (1.0, 0.0), t, 0.032))
        t += 0.032
    assert all(v >= 0.0 for v, _ in cmds)                    # never reversed blind
    assert fsm.state == "ERROR"                              # bounded


def test_collision_event_triggers_escape_and_replan():
    grid = empty_grid()
    fsm, logs, old = deliver_fsm(grid)
    try:
        sim = Unicycle((-2.0, 0.0, 0.0))
        drive(fsm, sim, lambda t: [], T=1.0)
        fsm.notify_collision(True, 1.0)
        v, w = fsm.update(sim.pose(), scan_boxes(sim.pose(), []), ANGLES, [], 1.05, 0.032)
        assert (v, w) == (0.0, 0.0)
        v, w = fsm.update(sim.pose(), scan_boxes(sim.pose(), []), ANGLES, [], 1.1, 0.032)
        assert v <= 0.0                                      # moves away from the front contact
        assert fsm.replan_reasons.get("COLLISION") == 1
        drive(fsm, sim, lambda t: [], t0=1.2, T=60.0)
        assert fsm.state == "DONE"
    finally:
        config.TASK_ZONE = old
        config.ZONE_OF.pop("T", None)


def test_replan_cooldown_limits_thrashing():
    grid = empty_grid()
    fsm, logs, old = deliver_fsm(grid)
    try:
        sim = Unicycle((-2.0, 0.0, 0.0))
        rng = np.random.default_rng(0)

        def flicker(t):                    # obstacle jumping around ahead
            y = rng.uniform(-0.5, 0.5)
            return [(0.0, 0.3, y - 0.2, y + 0.2)]
        drive(fsm, sim, flicker, T=10.0)
        ts = [e["t"] for e in fsm.replan_events if e["reason"] in ("PATH_BLOCKED", "DEVIATION")]
        gaps = np.diff(ts)
        assert len(gaps) == 0 or gaps.min() >= config.REPLAN_COOLDOWN - 1e-6
    finally:
        config.TASK_ZONE = old
        config.ZONE_OF.pop("T", None)


def test_follower_can_turn_away_from_a_wall_it_faces():
    """Pushed nose-first against a wall with the route behind: turning in
    place must be allowed (the old forward-cone check BLOCKED it forever)."""
    from waypoint_follower import WaypointFollower
    pose = (0.0, 0.0, 0.0)
    wall = (0.27, 0.4, -1.0, 1.0)                   # 0.27 m ahead (< STOP_DIST)
    scan = scan_boxes(pose, [wall])
    f = WaypointFollower([(0.0, 1.5), (0.0, 3.0)])  # route goes left
    v, w = f.update(pose, scan, ANGLES)
    assert f.status == "FOLLOWING" and v == 0.0 and abs(w) > 0.0
    # but a body-sweep conflict still blocks the rotation
    tight = scan_boxes(pose, [(0.15, 0.4, -1.0, 1.0)])
    f2 = WaypointFollower([(0.0, 1.5)])
    f2.update(pose, tight, ANGLES)
    assert f2.status == "BLOCKED"
