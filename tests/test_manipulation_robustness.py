"""Regression tests for the manipulation "freeze" fixes.

Each test reproduces one way the arm / gripper sequence could hang or kill
the controller before the fix, using the scripted FakeArmDriver:

* finger jitter under a force-limited motor never satisfied the per-step
  "still" test -> CLOSE_GRIPPER only ended at its timeout;
* a latched parcel can stop the arm a little short of the goal -> SETTLE
  timeout and a recover/retry loop;
* an exception inside a step (e.g. a NaN TCP) used to propagate and kill
  the Webots controller (robot frozen, no report);
* steps without an own timeout waited 60 s;
* the first STOW command at start-up failed on NaN encoders;
* faults now log a per-joint diagnosis (target / measured / torque / limits).
"""

import math

import numpy as np
import pytest

import config
from arm_fakes import FakeArmDriver, run
from gripper import Gripper
from pick_place import PickPlaceController, Step, build_manipulator

DONE = ("SUCCEEDED", "FAILED")


class JitterFingers(FakeArmDriver):
    """Fingers stalled on a parcel but jittering by +-0.3 mm every step."""

    def __init__(self):
        super().__init__()
        self.object_width = config.PKG_SIZE
        self.k = 0

    def read_fingers(self):
        self.k += 1
        j = 3e-4 if self.k % 2 else -3e-4
        return float(self.fingers[0]) + j, float(self.fingers[1]) - j


def test_gripper_close_finishes_despite_finger_jitter():
    drv = JitterFingers()
    g = Gripper(drv, log=lambda m: None)
    g.close(0.0)
    st, t = run(g.update, drv, max_t=10.0, done=lambda s: s in ("DONE", "FAULT"))
    assert st == "DONE", g.fault
    travel_time = config.GRIPPER_FINGER_TRAVEL / config.GRIPPER_FINGER_MAX_VEL
    assert t < travel_time + 1.0          # long before the CLOSING_TIMEOUT
    assert g.done_reason in ("stalled", "contact")
    assert g.assess().width_ok


def test_gripper_close_on_nothing_still_ends_by_stall():
    drv = FakeArmDriver()
    g = Gripper(drv, log=lambda m: None)
    g.close(0.0)
    st, t = run(g.update, drv, max_t=10.0, done=lambda s: s in ("DONE", "FAULT"))
    assert st == "DONE"
    assert g.assess().state == "MISSED"     # verification still says: nothing held


class SaggingArm(FakeArmDriver):
    """Shoulder stops `sag` rad short of every command (load / contact)."""

    def __init__(self, sag):
        super().__init__()
        self.sag = sag

    def step(self, dt):
        cmd = self.q_cmd.copy()
        self.q_cmd = cmd - np.array([self.sag, 0.0, 0.0]) * np.sign(cmd - self.q + 1e-12)
        super().step(dt)
        self.q_cmd = cmd


def _arm():
    from arm_controller import ArmController
    return ArmController


def test_arm_accepts_small_residual_once_stopped_under_load():
    drv = SaggingArm(sag=0.025)              # > ARM_JOINT_TOL, < ARM_JOINT_TOL_LOADED
    arm = _arm()(drv, log=lambda m: None)
    q_goal = list(drv.q + np.array([-0.3, 0.4, -0.2]))
    ok, _ = arm.move_joints(q_goal, 0.0)
    assert ok
    st, t = run(arm.update, drv, max_t=20.0, done=lambda s: s in ("DONE", "FAULT"))
    assert st == "DONE"
    assert "settled" in arm.settle_note
    assert t < arm.traj.T + config.ARM_MOTION_TIMEOUT_MARGIN


def test_arm_large_residual_still_faults_with_diagnosis():
    drv = SaggingArm(sag=0.10)               # clearly not at the goal
    logs = []
    arm = _arm()(drv, log=logs.append)
    q_goal = list(drv.q + np.array([-0.3, 0.4, -0.2]))
    arm.move_joints(q_goal, 0.0)
    st, _ = run(arm.update, drv, max_t=30.0, done=lambda s: s in ("DONE", "FAULT"))
    assert st == "FAULT" and "SETTLE_TIMEOUT" in arm.fault
    rows = arm.last_diagnosis
    assert [r["joint"] for r in rows] == ["shoulder", "elbow", "wrist"]
    assert abs(rows[0]["error"]) > 0.05
    assert any("shoulder" in m and "target=" in m for m in logs)


def test_tracking_fault_reports_the_blocked_joint():
    drv = FakeArmDriver()
    drv.freeze_joint = 1
    arm = _arm()(drv, log=lambda m: None)
    arm.move_joints(list(drv.q + np.array([0.0, 0.8, 0.0])), 0.0)
    st, _ = run(arm.update, drv, max_t=20.0, done=lambda s: s in ("DONE", "FAULT"))
    assert st == "FAULT" and "elbow" in arm.fault
    assert arm.last_diagnosis[1]["joint"] == "elbow"


def test_step_exception_fails_the_operation_instead_of_crashing():
    drv = FakeArmDriver()
    manip, _ = build_manipulator(drv, log=lambda m: None)

    def boom(t):
        raise RuntimeError("NaN TCP")
    manip._begin("demo", [Step("BAD_START", boom, lambda t: "DONE", timeout=1.0)], 0.0)
    assert manip.status == "FAILED" and "EXCEPTION" in manip.reason

    def bad_poll(t):
        raise ValueError("bad poll")
    manip._begin("demo", [Step("BAD_POLL", lambda t: (True, ""), bad_poll, timeout=1.0)], 0.0)
    assert manip.update(0.016) == "FAILED" and "bad poll" in manip.reason


def test_raise_target_with_invalid_sensors_fails_cleanly():
    drv = FakeArmDriver()
    manip, _ = build_manipulator(drv, log=lambda m: None)
    drv.invalid_joints = True
    manip.start_recover(0.0, keep_parcel=True)   # RETREAT/RAISE need the TCP
    st, _ = run(manip.update, drv, max_t=5.0, done=lambda s: s in DONE)
    assert st == "FAILED"


def test_arm_step_deadline_follows_the_planned_motion():
    drv = FakeArmDriver()
    drv.freeze_joint = 0
    manip, _ = build_manipulator(drv, log=lambda m: None)
    manip.start_stow(0.0)
    st, t = run(manip.update, drv, max_t=70.0, done=lambda s: s in DONE)
    # already at stow: success; now force a long blocked motion
    manip.arm.move_joints(list(drv.q + np.array([0.6, 0.0, 0.0])), t)
    manip._begin("blocked", [Step("MOVE", lambda tt: (True, ""), manip._arm_poll)], t)
    st, t2 = run(manip.update, drv, t0=t, max_t=70.0, done=lambda s: s in DONE)
    assert st == "FAILED"
    assert t2 - t < 20.0                     # was a flat 60 s step timeout


def test_stow_waits_for_valid_encoders_at_startup():
    drv = FakeArmDriver()
    drv.invalid_joints = True
    manip, _ = build_manipulator(drv, log=lambda m: None)
    manip.start_stow(0.0)
    assert manip.status == "RUNNING"         # did not fail on the NaN sample
    t, dt = 0.0, 0.016
    while t < 0.2:
        manip.update(t)
        drv.step(dt)
        t += dt
    drv.invalid_joints = False               # first sampling period done
    st, _ = run(manip.update, drv, t0=t, max_t=30.0, done=lambda s: s in DONE)
    assert st == "SUCCEEDED"


def test_stow_gives_up_if_encoders_never_become_valid():
    drv = FakeArmDriver()
    drv.invalid_joints = True
    manip, _ = build_manipulator(drv, log=lambda m: None)
    manip.start_stow(0.0)
    st, t = run(manip.update, drv, max_t=10.0, done=lambda s: s in DONE)
    assert st == "FAILED" and "WAIT_SENSORS" in manip.reason
    assert t <= config.ARM_SENSOR_WAIT_TIMEOUT + 0.1


def test_one_arm_update_per_control_step():
    drv = FakeArmDriver()
    manip, _ = build_manipulator(drv, log=lambda m: None)
    calls = []
    orig = manip.arm.update

    def counting(t):
        calls.append(t)
        return orig(t)
    manip.arm.update = counting
    manip.start_stow(0.0)
    for k in range(5):
        manip.update(k * 0.016)
        drv.step(0.016)
    assert len(calls) == len(set(calls))     # never twice for the same t
