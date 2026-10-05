"""WaypointFollower tests with a simulated unicycle (no Webots)."""

import math

import pytest

import config
import kinematics as k
from controller import WaypointFollower


class Unicycle:
    """Ideal kinematic simulation of the robot body."""

    def __init__(self, x, y, theta):
        self.x, self.y, self.theta = x, y, theta

    def step(self, v, w, dt=1.0 / 60.0):
        self.theta = k.wrap_to_pi(self.theta + w * dt)
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

    def pose(self):
        return self.x, self.y, self.theta


def follow(follower, sim, dt=1.0 / 60.0, max_steps=6000):
    for _ in range(max_steps):
        v, w = follower.update(sim.pose())
        if follower.status == "REACHED":
            return True
        if follower.status == "BLOCKED":
            return False
        sim.step(v, w, dt)
    return False


def test_reaches_single_waypoint():
    sim = Unicycle(0.0, 0.0, 0.0)
    f = WaypointFollower([(2.0, 1.0)])
    assert follow(f, sim)
    x, y, _ = sim.pose()
    assert math.hypot(x - 2.0, y - 1.0) < config.GOAL_TOLERANCE


def test_reaches_multivaypoint_path():
    sim = Unicycle(0.0, 0.0, 0.0)
    wps = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 2.0)]
    f = WaypointFollower(wps)
    assert follow(f, sim)
    x, y, _ = sim.pose()
    assert math.hypot(x, y - 2.0) < config.GOAL_TOLERANCE
    assert f.idx >= len(wps) - 1


def test_starts_misaligned_rotates_first():
    sim = Unicycle(0.0, 0.0, math.pi / 2)       # facing +y, target +x
    f = WaypointFollower([(2.0, 0.0)])
    v0, w0 = f.update(sim.pose())
    assert v0 == 0.0                            # turn in place first
    assert abs(w0) > 0.1
    assert follow(f, sim)


def test_lidar_block_sets_status():
    f = WaypointFollower([(2.0, 0.0)])
    n = 360
    angles = [-math.pi + i * 2 * math.pi / n for i in range(n)]
    scan = [4.0] * n
    # obstacle dead ahead (bearing 0) at 0.2 m < STOP_DIST
    idx = n // 2
    for i in range(idx - 5, idx + 6):
        scan[i] = 0.2
    v, w = f.update((0.0, 0.0, 0.0), scan, angles)
    assert v == 0.0 and w == 0.0
    assert f.status == "BLOCKED"


def test_no_false_block_when_far():
    f = WaypointFollower([(2.0, 0.0)])
    n = 360
    angles = [-math.pi + i * 2 * math.pi / n for i in range(n)]
    scan = [0.5] * n                            # 0.5 m everywhere: > STOP_DIST
    f.update((0.0, 0.0, 0.0), scan, angles)
    assert f.status == "FOLLOWING"


def test_slowdown_near_goal():
    sim = Unicycle(1.7, 0.0, 0.0)               # 0.3 m from the waypoint
    f = WaypointFollower([(2.0, 0.0)])
    v, _ = f.update(sim.pose())
    assert 0.0 < v <= config.V_MAX * (0.3 / config.SLOWDOWN_DIST) + 1e-9


def test_crosstrack_error():
    sim = Unicycle(1.0, 0.5, 0.0)               # 0.5 m off the x-axis line
    f = WaypointFollower([(1.0, 0.0), (3.0, 0.0)])
    f.idx = 1
    v, w = f.update(sim.pose())
    assert f.cross_track_err == pytest.approx(0.5, abs=0.01)
