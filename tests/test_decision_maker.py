"""FSM tests with fake inputs — happy path, timeout ERROR, blocked path.

The FSM drives a simulated unicycle; the fake planner returns simple
waypoint polylines, the fake estimator a fixed package position.
"""

import math

import pytest

import config
import kinematics as k
from controller import WaypointFollower
from decision_maker import DecisionMaker


class Unicycle:
    def __init__(self, x, y, theta):
        self.x, self.y, self.theta = x, y, theta

    def step(self, v, w, dt):
        self.theta = k.wrap_to_pi(self.theta + w * dt)
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

    def pose(self):
        return self.x, self.y, self.theta


class FakeDet:
    """Duck-typed Detection for the target package."""
    pkg_id = config.TASK_PACKAGE
    color = config.TASK_PACKAGE_COLOR


def straight_planner():
    """Fake planner: straight line start->goal at 0.5 m spacing."""
    def plan(start, goal):
        dx, dy = goal[0] - start[0], goal[1] - start[1]
        dist = math.hypot(dx, dy)
        n = max(1, int(dist / 0.5))
        wps = [(start[0] + dx * i / n, start[1] + dy * i / n)
               for i in range(1, n + 1)]
        info = {"n_waypoints": n, "length_m": dist, "runtime_ms": 0.1,
                "expansions": 10}
        return wps, info
    return plan


def fixed_estimator(x, y):
    def est(det, pose, scan, angles):
        return (x, y, 0.0, math.hypot(x - pose[0], y - pose[1]))
    return est


def run_fsm(fsm, sim, dt=0.05, max_s=120.0, detections=None):
    """Run the FSM loop; returns (states_seen, final_state)."""
    t = 0.0
    states = [fsm.state]
    while t < max_s:
        dets = detections() if callable(detections) else (detections or [])
        v, w = fsm.update(sim.pose(), None, None, dets, t, dt)
        if states[-1] != fsm.state:
            states.append(fsm.state)
        if fsm.state in ("DONE", "ERROR"):
            return states, fsm.state
        sim.step(v, w, dt)
        t += dt
    return states, fsm.state


def test_happy_path_reaches_done():
    sim = Unicycle(0.0, -0.3, 0.0)
    pkg = (-3.1, -0.5)
    fsm = DecisionMaker(straight_planner(), fixed_estimator(*pkg))
    states, final = run_fsm(fsm, sim, detections=[FakeDet()])
    assert final == "DONE"
    for s in ("IDLE", "FIND_PACKAGE", "NAVIGATE", "PICK", "PLAN_DELIVERY",
              "DELIVER", "DONE"):
        assert s in states, f"missing state {s}: {states}"
    # mission report filled in
    assert fsm.mission_report is not None
    assert fsm.mission_report["replans"] == 0
    assert fsm.mission_report["path_length_m"] > 0


def test_package_never_found_error():
    sim = Unicycle(0.0, -0.3, 0.0)
    fsm = DecisionMaker(straight_planner(), fixed_estimator(-3.1, -0.5))
    states, final = run_fsm(fsm, sim, detections=[])   # never any detection
    assert final == "ERROR"
    assert "FIND_PACKAGE" in states


def test_planner_failure_navigates_to_error():
    sim = Unicycle(0.0, -0.3, 0.0)

    def failing_planner(start, goal):
        return None, {"found": False}

    # place the package right next to the robot so FIND succeeds fast
    fsm = DecisionMaker(failing_planner, fixed_estimator(0.4, -0.3))
    states, final = run_fsm(fsm, sim)
    assert final == "ERROR"


def test_transitions_logged(capsys):
    sim = Unicycle(0.0, -0.3, 0.0)
    fsm = DecisionMaker(straight_planner(), fixed_estimator(-1.0, -0.5))
    run_fsm(fsm, sim, detections=[FakeDet()])
    out = capsys.readouterr().out
    assert "STATE: IDLE -> FIND_PACKAGE" in out
    assert "STATE: FIND_PACKAGE -> NAVIGATE" in out
    assert "STATE: NAVIGATE -> PICK" in out
    assert "PICK placeholder" in out
    assert "MISSION REPORT" in out


def test_blocked_in_navigate_is_error():
    """When the follower reports BLOCKED the FSM stops safely (the final-eval
    re-planner will take over this hook)."""
    sim = Unicycle(0.0, -0.3, 0.0)
    fsm = DecisionMaker(straight_planner(), fixed_estimator(-1.0, -0.5))

    real_update = WaypointFollower.update

    def blocked_update(self, pose, scan=None, angles=None):
        self.status = "BLOCKED"
        return 0.0, 0.0

    import controller
    controller.WaypointFollower.update = blocked_update
    try:
        states, final = run_fsm(fsm, sim, detections=[FakeDet()])
    finally:
        controller.WaypointFollower.update = real_update
    assert final == "ERROR"
