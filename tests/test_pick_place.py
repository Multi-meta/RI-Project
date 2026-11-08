"""Pick / place sequences: nominal flow and every failure branch.

Nominal flows run on the kinematic world (tools/offline_mission_sim.py) with
the robot already docked; failure branches use the scripted FakeArmDriver.
Key property: no sequence reports SUCCEEDED unless its sensor check passed.
"""

import math

import pytest

import config
import shelving
from arm_fakes import FakeArmDriver, run
from offline_mission_sim import KinematicArmDriver, KinematicWorld
from pick_place import build_manipulator

DONE = ("SUCCEEDED", "FAILED")


def docked_world():
    """Robot docked in front of P3 (west wall), facing west, at the standoff."""
    w = KinematicWorld()
    p3 = w.parcels["PKG_P3"]
    x = p3.pos[0] + config.PKG_SIZE / 2 + config.ARM_DOCK_PICK_STANDOFF
    w.pose = [x, p3.pos[1], math.pi]
    drv = KinematicArmDriver(w)
    manip, _ = build_manipulator(drv, log=lambda m: None)
    return w, drv, manip


def step_all(manip, drv, t0=0.0, dt=0.016, max_t=90.0):
    t = t0
    while t < t0 + max_t:
        st = manip.update(t)
        if st in DONE:
            return st, t
        drv.step(dt)
        t += dt
    return manip.status, t


def test_pick_and_verify_attachment_lifts_the_parcel():
    w, drv, manip = docked_world()
    manip.start_pick(0.0, shelving.pick_tcp_for_parcel())
    st, t = step_all(manip, drv)
    assert st == "SUCCEEDED", manip.reason
    p3 = w.parcels["PKG_P3"]
    assert p3.attached and p3.pos[2] > config.PKG_SIZE / 2 + 0.04     # lift check
    manip.start_verify_attachment(t)
    st, t = step_all(manip, drv, t)
    assert st == "SUCCEEDED", manip.reason
    assert manip.last_assessment.state == "HOLDING"
    # carried over the body, above the LiDAR plane
    assert p3.pos[2] - config.PKG_SIZE / 2 > config.LIDAR_MOUNT_Z
    steps = [h[2] for h in manip.history if h[1] == "pick"]
    assert steps[:4] == ["OPEN_GRIPPER", "PRE_GRASP", "DESCEND", "ADVANCE"]
    assert "LATCH" in steps and "CHECK_GRASP" in steps


def test_full_pick_then_place_on_floor_and_verify_release():
    """Pick, then put the parcel back on the floor at the same spot."""
    w, drv, manip = docked_world()
    g = shelving.pick_tcp_for_parcel()
    manip.start_pick(0.0, g)
    st, t = step_all(manip, drv)
    manip.start_verify_attachment(t)
    st, t = step_all(manip, drv, t)
    floor = {"x": g["x"], "pre_x": g["x"] - 0.06,
             "z_hover": g["z"] + config.ARM_PLACE_HOVER,
             "z_release": g["z"] + config.ARM_PLACE_RELEASE_DROP,
             "retract_x": g["x"] - config.ARM_RETRACT_DIST}
    manip.start_place(t, floor)
    st, t = step_all(manip, drv, t)
    assert st == "SUCCEEDED", manip.reason
    manip.start_verify_delivery(t, floor)
    st, t = step_all(manip, drv, t)
    assert st == "SUCCEEDED", manip.reason
    p3 = w.parcels["PKG_P3"]
    assert not p3.attached
    assert p3.pos[2] == pytest.approx(config.PKG_SIZE / 2)


def test_missed_parcel_fails_at_palm_check_never_succeeds():
    w, drv, manip = docked_world()
    del w.parcels["PKG_P3"]                       # nothing to pick
    manip.start_pick(0.0, shelving.pick_tcp_for_parcel())
    st, _ = step_all(manip, drv)
    assert st == "FAILED" and "PARCEL_NOT_AT_PALM" in manip.reason


def scripted(**kw):
    drv = FakeArmDriver(**kw)
    manip, _ = build_manipulator(drv, log=lambda m: None)
    return drv, manip


def test_grasp_missed_when_fingers_close_completely():
    drv, manip = scripted()
    drv.palm_range = 0.01                          # something at the palm ...
    drv.object_width = None                        # ... but not between the fingers
    manip.start_pick(0.0, shelving.pick_tcp_for_parcel())
    st, _ = run(manip.update, drv, max_t=90, done=lambda s: s in DONE)
    assert st == "FAILED" and "GRASP_MISSED" in manip.reason


def test_latch_absent_is_an_attachment_failure():
    drv, manip = scripted()
    drv.palm_range, drv.object_width, drv.presence = 0.01, config.PKG_SIZE, 0
    manip.start_pick(0.0, shelving.pick_tcp_for_parcel())
    st, _ = run(manip.update, drv, max_t=90, done=lambda s: s in DONE)
    assert st == "FAILED" and "LATCH_PEER_NOT_PRESENT" in manip.reason


def test_latch_that_never_locks_times_out():
    drv, manip = scripted()
    drv.palm_range, drv.object_width, drv.presence = 0.01, config.PKG_SIZE, 1
    drv.lock_works = False
    manip.start_pick(0.0, shelving.pick_tcp_for_parcel())
    st, _ = run(manip.update, drv, max_t=90, done=lambda s: s in DONE)
    assert st == "FAILED" and manip.reason.startswith("LATCH")


def test_parcel_lost_after_lift_fails_verify_attachment():
    drv, manip = scripted()
    drv.palm_range, drv.object_width, drv.presence = 0.01, config.PKG_SIZE, 1
    manip.start_pick(0.0, shelving.pick_tcp_for_parcel())
    st, t = run(manip.update, drv, max_t=90, done=lambda s: s in DONE)
    assert st == "SUCCEEDED"
    drv.object_width, drv.palm_range, drv.locked = None, 0.5, False   # it fell
    manip.start_verify_attachment(t)
    st, _ = run(manip.update, drv, t0=t, max_t=30, done=lambda s: s in DONE)
    assert st == "FAILED" and "ATTACHMENT_LOST" in manip.reason


def holding_driver():
    drv, manip = scripted()
    drv.palm_range, drv.object_width, drv.presence, drv.locked = 0.01, config.PKG_SIZE, 1, True
    drv.finger_cmd = config.GRIPPER_FINGER_TRAVEL
    drv.fingers[:] = config.GRIPPER_OPEN_HALF_WIDTH - config.PKG_SIZE / 2
    return drv, manip


def test_blocked_slot_is_reported_as_slot_blocked():
    drv, manip = holding_driver()
    slot = shelving.slots_for_zone("B")[0]
    place = shelving.place_tcp_for_slot(slot)
    manip.start_place(0.0, place)

    def upd(t):
        if manip.step_name == "INSERT":
            drv.freeze_joint = 0                   # the arm hits something
        return manip.update(t)
    st, _ = run(upd, drv, max_t=60, done=lambda s: s in DONE)
    assert st == "FAILED" and "SLOT_BLOCKED" in manip.reason


def test_release_verification_catches_parcel_still_attached():
    drv, manip = holding_driver()
    slot = shelving.slots_for_zone("B")[0]
    place = shelving.place_tcp_for_slot(slot)
    drv.unlock_works = False                       # latch jammed
    manip.start_place(0.0, place)
    st, _ = run(manip.update, drv, max_t=60, done=lambda s: s in DONE)
    assert st == "FAILED" and "UNLATCH" in manip.reason


def test_release_verification_catches_parcel_not_in_slot():
    drv, manip = holding_driver()
    slot = shelving.slots_for_zone("B")[0]
    place = shelving.place_tcp_for_slot(slot)
    manip.start_place(0.0, place)
    st, t = run(manip.update, drv, max_t=60, done=lambda s: s in DONE)
    assert st == "SUCCEEDED"
    drv.object_width = None
    drv.palm_range = 0.5                           # parcel fell off the shelf
    manip.start_verify_delivery(t, place)
    st, _ = run(manip.update, drv, t0=t, max_t=60, done=lambda s: s in DONE)
    assert st == "FAILED" and "PARCEL_NOT_IN_SLOT" in manip.reason


def test_release_verification_passes_when_range_matches():
    drv, manip = holding_driver()
    slot = shelving.slots_for_zone("B")[0]
    place = shelving.place_tcp_for_slot(slot)
    manip.start_place(0.0, place)
    st, t = run(manip.update, drv, max_t=60, done=lambda s: s in DONE)
    drv.object_width = None
    drv.palm_range = config.GRIPPER_PALM_GAP + config.ARM_RETRACT_DIST
    manip.start_verify_delivery(t, place)
    st, _ = run(manip.update, drv, t0=t, max_t=60, done=lambda s: s in DONE)
    assert st == "SUCCEEDED", manip.reason
    assert manip.last_check["CHECK_RELEASED"].startswith("palm_range=7.0cm")


def test_recover_returns_arm_to_stow_after_failure():
    drv, manip = scripted()
    manip.start_pick(0.0, shelving.pick_tcp_for_parcel())
    run(manip.update, drv, max_t=90, done=lambda s: s in DONE)     # fails (no parcel)
    assert manip.status == "FAILED"
    manip.start_recover(100.0)
    st, _ = run(manip.update, drv, t0=100.0, max_t=60, done=lambda s: s in DONE)
    assert st == "SUCCEEDED", manip.reason
    assert manip.arm.tcp() == pytest.approx(config.ARM_STOW_TCP, abs=0.01)
    assert not drv.locked and manip.gripper.is_open()
