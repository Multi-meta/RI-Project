"""ArmController (motion + safety) and Gripper (state + grasp fusion) tests."""

import math

import numpy as np
import pytest

import config
from arm_controller import ArmController
from arm_fakes import FakeArmDriver, run
from arm_kinematics import PlanarPitchArm
from gripper import Gripper

KIN = PlanarPitchArm.from_config()


def make_arm(**kw):
    drv = FakeArmDriver(**kw)
    return ArmController(drv, KIN, log=lambda m: None), drv


def test_move_line_reaches_target_and_follows_a_straight_line():
    arm, drv = make_arm()
    ok, why = arm.move_line(0.30, 0.30, 0.0)
    assert ok, why
    xs = []

    def upd(t):
        st = arm.update(t)
        xs.append(KIN.forward(drv.q_cmd)["TCP"])
        return st
    st, _ = run(upd, drv, done=lambda s: s in ("DONE", "FAULT"))
    assert st == "DONE"
    assert arm.tcp() == pytest.approx((0.30, 0.30), abs=0.005)
    # commanded TCP stays on the segment stow -> target (straight line)
    x0, z0 = config.ARM_STOW_TCP
    for x, z in xs:
        cross = (x - x0) * (0.30 - z0) - (z - z0) * (0.30 - x0)
        assert abs(cross) / math.hypot(0.30 - x0, 0.30 - z0) < 0.005


def test_joint_move_is_smooth_and_within_velocity_limits():
    arm, drv = make_arm()
    q_goal = KIN.inverse(0.38, 0.265, 0.0)
    ok, _ = arm.move_joints(q_goal, 0.0)
    assert ok
    prev, dt, vmax_seen = np.array(drv.q_cmd), 0.016, 0.0
    t = 0.0
    while arm.update(t) == "MOVING" and t < 30:
        v = np.abs(np.array(drv.q_cmd) - prev) / dt
        vmax_seen = max(vmax_seen, float(v.max()))
        prev = np.array(drv.q_cmd)
        drv.step(dt)
        t += dt
    assert arm.status == "DONE"
    assert vmax_seen <= min(config.ARM_JOINT_MAX_VEL) + 1e-6


def test_unreachable_targets_are_rejected_before_moving():
    arm, drv = make_arm()
    ok, why = arm.move_line(1.2, 0.3, 0.0)
    assert not ok and why == "OUT_OF_REACH"
    ok, why = arm.move_joints((5.0, 0.0, 0.0), 0.0)
    assert not ok and why == "JOINT_LIMIT"
    assert arm.status == "IDLE"


def test_blocked_joint_raises_tracking_fault_and_holds():
    arm, drv = make_arm()
    drv.freeze_joint = 0
    arm.move_line(0.38, 0.265, 0.0, name="INSERT")
    st, _ = run(arm.update, drv, done=lambda s: s in ("DONE", "FAULT"))
    assert st == "FAULT"
    assert "TRACKING_ERROR(shoulder" in arm.fault and "INSERT" in arm.fault


def test_invalid_joint_sensor_is_a_fault_not_a_success():
    arm, drv = make_arm()
    ok, why = arm.move_line(0.3, 0.3, 0.0)
    assert ok
    drv.invalid_joints = True
    st, _ = run(arm.update, drv, done=lambda s: s in ("DONE", "FAULT"))
    assert st == "FAULT" and "INVALID_JOINT_SENSOR" in arm.fault
    arm2, drv2 = make_arm()
    drv2.invalid_joints = True
    assert arm2.move_line(0.3, 0.3, 0.0) == (False, "INVALID_JOINT_SENSOR")


# ------------------------------------------------------------------ gripper
def make_gripper():
    drv = FakeArmDriver()
    return Gripper(drv, log=lambda m: None), drv


def test_width_from_finger_travel():
    assert Gripper.width_from_travel(0.0, 0.0) == pytest.approx(2 * config.GRIPPER_OPEN_HALF_WIDTH)
    assert Gripper.width_from_travel(0.03, 0.03) == pytest.approx(0.20)


def test_close_on_parcel_stalls_and_assesses_holding():
    g, drv = make_gripper()
    drv.object_width = config.PKG_SIZE
    drv.palm_range = 0.01
    drv.presence = 1
    g.close(0.0)
    st, _ = run(g.update, drv, done=lambda s: s in ("DONE", "FAULT"))
    assert st == "DONE"
    a = g.assess()
    assert a.width == pytest.approx(config.PKG_SIZE, abs=1e-3)
    assert a.state == "HOLDING" and a.holding
    assert a.width_ok and a.force_ok and a.palm_ok


def test_close_on_nothing_is_a_missed_grasp():
    g, drv = make_gripper()
    drv.palm_range = 0.01
    g.close(0.0)
    run(g.update, drv, done=lambda s: s in ("DONE", "FAULT"))
    a = g.assess()
    assert not a.holding and a.state == "MISSED"
    assert a.width == pytest.approx(2 * config.GRIPPER_OPEN_HALF_WIDTH
                                    - 2 * config.GRIPPER_FINGER_TRAVEL, abs=1e-3)


def test_fusion_requires_palm_and_force_or_latch():
    g, drv = make_gripper()
    drv.fingers[:] = config.GRIPPER_OPEN_HALF_WIDTH - config.PKG_SIZE / 2   # width = parcel
    drv.finger_cmd = config.GRIPPER_FINGER_TRAVEL
    drv.object_width = config.PKG_SIZE
    drv.palm_range = 0.20                       # but the palm sees nothing
    a = g.assess()
    assert not a.holding and not a.palm_ok and a.state == "PARTIAL"
    drv.palm_range = 0.01
    drv.grip_force = 0.0                        # no force reading ...
    assert not g.assess().holding
    drv.locked = True                           # ... but the latch is locked
    assert g.assess().holding


def test_invalid_sensor_gives_unknown():
    g, drv = make_gripper()
    drv.palm_range = float("nan")
    a = g.assess()
    assert a.state == "UNKNOWN" and not a.holding


def test_attach_requires_peer_presence():
    g, drv = make_gripper()
    drv.presence = 0
    assert g.attach() == (False, "LATCH_PEER_NOT_PRESENT")
    assert not drv.locked
    drv.presence = 1
    assert g.attach() == (True, "")
    assert g.latched()
    g.release()
    assert not g.latched()


def test_open_reports_done_only_when_fully_open():
    g, drv = make_gripper()
    drv.fingers[:] = 0.05
    g.open(0.0)
    st, _ = run(g.update, drv, done=lambda s: s in ("DONE", "FAULT"))
    assert st == "DONE" and g.is_open()


def test_gripper_timeout_is_a_fault():
    g, drv = make_gripper()

    class Stuck(FakeArmDriver):
        def step(self, dt):
            pass                                # fingers never move
    stuck = Stuck()
    stuck.fingers[:] = 0.05
    g2 = Gripper(stuck, log=lambda m: None)
    g2.open(0.0)
    st, _ = run(g2.update, stuck, max_t=20, done=lambda s: s in ("DONE", "FAULT"))
    assert st == "FAULT" and "TIMEOUT" in g2.fault


@pytest.mark.parametrize("target", [(0.40, 0.27), (0.13, 0.525), (0.38, 0.265)])
def test_cartesian_lines_respect_joint_velocity_limits(target):
    """Even through the folded stow region (high joint sensitivity)."""
    arm, drv = make_arm()
    ok, why = arm.move_line(*target, 0.0)
    assert ok, why
    dt, t = 0.008, 0.0
    prev = np.array(arm.traj.sample(0.0))
    vmax = np.array(config.ARM_JOINT_MAX_VEL)
    while t < arm.traj.T:
        t += dt
        cur = np.array(arm.traj.sample(t))
        assert np.all(np.abs(cur - prev) / dt <= vmax + 1e-6)
        prev = cur
