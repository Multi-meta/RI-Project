"""End-to-end regression on the offline KINEMATIC simulator (not Webots).

The real FSM + EKF localisation + dynamic map + speed governor + arm code run
complete pick-and-place missions with obstacles and a displacement. These are
regression guards for the behaviour reported by teammates (robot replanning
from the old position after being displaced / hitting something, slow
missions, freezes); they do not replace a Webots run.
"""

import math

import pytest

import config
from offline_mission_sim import MovingBox, run_mission


def _contacts(res):
    return res["collisions"]["front"] + res["collisions"]["rear"]


def test_full_stack_mission_with_ekf_and_governor():
    res = run_mission("P3", "B", navigation="full", localization="ekf", governor=True)
    assert res["final_state"] == "DONE"
    ok, err = res["audit"]
    assert ok and err["xy"] < 0.03
    assert _contacts(res) == 0
    assert res["localization"]["rmse_xy"] < 0.02


def test_static_obstacle_detour_without_contact():
    box = MovingBox(0.8, -0.45, 0.3, 0.3, t_on=40.0)
    res = run_mission("P3", "B", obstacles=[box])
    assert res["final_state"] == "DONE"
    assert _contacts(res) == 0
    assert res["replan_reasons"].get("PATH_BLOCKED", 0) >= 1
    assert res["min_clearance"] > 0.1


def test_displacement_relocalises_and_replans_from_the_new_pose():
    res = run_mission("P3", "B", displace=(46.0, 0.3, 0.5, 0.6), gps_noise=0.02,
                      imu_noise=0.005, odom_scale_error=0.01, max_time=300)
    assert res["final_state"] == "DONE"
    assert res["localization"]["summary"]["relocalizations"] == 1
    assert res["replan_reasons"].get("POSE_JUMP") == 1
    assert any("trigger POSE_JUMP" in m for m in res["log"])
    ev = [e for e in res["fsm"].replan_events if e["reason"] == "POSE_JUMP"][0]
    # replanned from (close to) the TRUE displaced position, not the old one:
    # the robot was at y ~ -0.43 and was pushed +0.5 m in y
    assert ev["start"][1] > -0.2


def test_crossing_cart_is_yielded_to(monkeypatch):
    monkeypatch.setattr(config, "YIELD_WAIT_TIME", 8.0)      # the shipped value
    cart = MovingBox(1.6, -1.3, 0.4, 0.3, vy=0.4, travel=1.5)
    res = run_mission("P3", "B", obstacles=[cart])
    assert res["final_state"] == "DONE"
    assert _contacts(res) == 0
    assert any("MOVING obstacle" in m for m in res["log"])


def test_governor_makes_navigation_faster_than_fixed_speed():
    fast = run_mission("P1", "A", governor=True)
    slow = run_mission("P1", "A", governor=False)
    assert fast["final_state"] == slow["final_state"] == "DONE"
    assert fast["time"] < slow["time"]
    assert _contacts(fast) == 0


def test_base_never_moves_during_manipulation_full_stack():
    # run_mission asserts (v, w) == (0, 0) in every ARM_* / VERIFY_* state
    res = run_mission("P2", "B")
    assert res["final_state"] == "DONE"
