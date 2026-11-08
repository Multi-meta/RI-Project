"""Mission FSM with the manipulator: transitions, retries, failure handling.

End-to-end runs use the offline kinematic world (NOT physics); the failure
branches use a scripted manipulator so each FSM decision is tested in
isolation. The invariant checked everywhere: the FSM never reaches DONE
unless the delivery was verified by sensors, and the base never moves while
the arm works.
"""

import math

import pytest

import config
import shelving
from decision_maker import DecisionMaker
from offline_mission_sim import run_mission

ARM_STATES = ("ARM_STOW", "ARM_PICK", "VERIFY_ATTACHMENT", "ARM_PLACE",
              "VERIFY_DELIVERY", "ARM_RECOVER")


# ------------------------------------------------------- end-to-end (kinematic)
def test_store_mission_p3_to_zone_b_reaches_done_with_parcel_in_slot():
    r = run_mission("P3", "B")
    assert r["final_state"] == "DONE"
    assert r["states"] == ["IDLE", "ARM_STOW", "FIND_PACKAGE", "NAVIGATE", "DOCK_PICK",
                           "ARM_PICK", "VERIFY_ATTACHMENT", "PLAN_DELIVERY", "DELIVER",
                           "DOCK_PLACE", "ARM_PLACE", "VERIFY_DELIVERY", "DONE"]
    rep = r["report"]
    assert rep["result"] == "DELIVERED" and rep["delivery_verified_by_sensors"]
    assert rep["slot"] == "RB-L1-B1" and rep["grasp_attempts"] == 1
    ok, err = r["audit"]
    assert ok and err["xy"] < 0.01
    assert r["fsm"].carrying is False


def test_zone_a_and_upper_level_when_lower_level_is_full():
    r = run_mission("P1", "A", occupied_slots=("RA-L1-B1", "RA-L1-B2"))
    assert r["final_state"] == "DONE"
    assert r["slot"] == "RA-L2-B1" and r["audit"][0]
    assert r["parcel_pos"][2] == pytest.approx(config.DEST_RACK_LEVEL_TOPS[1] + config.PKG_SIZE / 2)


def test_full_rack_is_an_error_not_a_success():
    full = tuple(s.slot_id for s in shelving.slots_for_zone("B"))
    r = run_mission("P3", "B", occupied_slots=full)
    assert r["final_state"] == "ERROR"
    assert "NO_FREE_SLOT" in r["report"]["failure_reason"]
    assert r["report"]["result"] == "FAILED"


@pytest.mark.parametrize("idx", [1, 4, 8])
def test_campaign_start_poses(idx):
    r = run_mission("P3", "B", start=config.CAMPAIGN_STARTS[idx])
    assert r["final_state"] == "DONE", r["report"]
    assert r["audit"][0]


def test_carrying_flag_only_between_verified_pick_and_release():
    r = run_mission("P3", "B", record=True)
    fsm = r["fsm"]
    # replay: carrying must be False before VERIFY_ATTACHMENT finished
    assert fsm.mission_report["delivery_verified_by_sensors"]
    states_seen = [row[4] for row in r["trajectory"]]
    assert states_seen.index("PLAN_DELIVERY") > states_seen.index("VERIFY_ATTACHMENT")


# ----------------------------------------------------- scripted manipulator
class ScriptedManip:
    """Stand-in for PickPlaceController: each start_* consumes one outcome."""

    def __init__(self, outcomes):
        self.outcomes = list(outcomes)       # e.g. ["SUCCEEDED", "FAILED:..."]
        self.status, self.reason, self.operation = "IDLE", "", ""
        self.started = []
        self.last_check = {}

    def _start(self, op):
        self.operation = op
        self.started.append(op)
        out = self.outcomes.pop(0) if self.outcomes else "SUCCEEDED"
        self.status, _, self.reason = out.partition(":")

    def start_stow(self, t): self._start("stow")
    def start_pick(self, t, g): self._start("pick")
    def start_verify_attachment(self, t): self._start("verify_attachment")
    def start_place(self, t, p): self._start("place")
    def start_verify_delivery(self, t, p): self._start("verify_delivery")
    def start_recover(self, t, keep_parcel=False, retreat_x=None):
        self._start("recover_keep" if keep_parcel else "recover")

    def update(self, t):
        return self.status


class InstantDocker:
    def __init__(self, ok=True, rng=None):
        self.status = "DONE" if ok else "FAILED"
        self.reason = "" if ok else "LATERAL_ERROR"
        self.measured_range = rng
        self.range_source = "lidar"
        self.final_lateral_error = 0.0
        self.final_heading_error = 0.0
        self.name = "DOCK"
        self.refined_center = None

    def update(self, *a):
        return 0.0, 0.0


def fsm_with(outcomes, monkeypatch, dock_ok=True):
    import decision_maker as dm

    def fake_docker(line, **kw):
        rng = (config.ARM_DOCK_PICK_STANDOFF if line is None
               else config.ARM_DOCK_PLACE_STANDOFF)
        return InstantDocker(dock_ok, rng)
    monkeypatch.setattr(dm, "Docker", fake_docker)
    manip = ScriptedManip(outcomes)
    planner = lambda s, g: ([g], {"length_m": 1.0, "n_waypoints": 1})  # noqa: E731
    est = lambda d, p, s, a: (-3.1, -0.5, 0.0, 1.0)                     # noqa: E731
    fsm = DecisionMaker(planner, est, manipulator=manip, log=lambda m: None)
    return fsm, manip


def drive(fsm, steps=400):
    class Det:
        pkg_id = config.TASK_PACKAGE
    seen = [fsm.state]
    for i in range(steps):
        pose = (0.0, 0.0, 0.0)
        if fsm.state == "NAVIGATE" and fsm.approach_pose is not None:
            pose = (*fsm.approach_pose, math.pi)           # teleport: arrived
        if fsm.state == "DELIVER" and fsm.follower is not None:
            fsm.follower.status = "REACHED"
        v, w = fsm.update(pose, None, None, [Det()], i * 0.1, 0.1)
        if fsm.state in ARM_STATES:
            assert (v, w) == (0.0, 0.0)
        if seen[-1] != fsm.state:
            seen.append(fsm.state)
        if fsm.state in ("DONE", "ERROR"):
            break
    return seen


def test_pick_failure_recovers_and_retries_then_succeeds(monkeypatch):
    fsm, m = fsm_with(["SUCCEEDED", "FAILED:CHECK_GRASP: GRASP_MISSED"], monkeypatch)
    seen = drive(fsm)
    assert seen[-1] == "DONE"
    assert m.started[:4] == ["stow", "pick", "recover", "pick"]
    assert fsm.grasp_attempts == 2
    assert "ARM_RECOVER" in seen


def test_repeated_pick_failure_ends_in_error_never_done(monkeypatch):
    fsm, m = fsm_with(["SUCCEEDED", "FAILED:CHECK_GRASP: GRASP_MISSED",
                       "SUCCEEDED", "FAILED:CHECK_GRASP: GRASP_MISSED"], monkeypatch)
    seen = drive(fsm)
    assert seen[-1] == "ERROR" and "DONE" not in seen
    assert m.started[-1] == "recover"                 # arm stowed before ERROR
    assert fsm.mission_report["result"] == "FAILED"
    assert "GRASP_MISSED" in fsm.mission_report["failure_reason"]


def test_attachment_lost_is_retried(monkeypatch):
    fsm, m = fsm_with(["SUCCEEDED", "SUCCEEDED", "FAILED:CHECK_ATTACHED: ATTACHMENT_LOST"],
                      monkeypatch)
    seen = drive(fsm)
    assert seen[-1] == "DONE"
    assert m.started[:5] == ["stow", "pick", "verify_attachment", "recover", "pick"]


def test_blocked_slot_keeps_parcel_and_uses_next_slot(monkeypatch):
    fsm, m = fsm_with(["SUCCEEDED"] * 3 + ["FAILED:INSERT: SLOT_BLOCKED: TRACKING_ERROR"],
                      monkeypatch)
    seen = drive(fsm)
    assert seen[-1] == "DONE"
    assert "recover_keep" in m.started                # parcel stays latched
    assert fsm.occupancy.occupied["RB-L1-B1"] == "blocked"
    assert fsm.mission_report["slot"] == "RB-L1-B2"


def test_unverified_delivery_is_an_error(monkeypatch):
    fsm, m = fsm_with(["SUCCEEDED"] * 4 + ["FAILED:CHECK_RELEASED: PARCEL_NOT_IN_SLOT"],
                      monkeypatch)
    seen = drive(fsm)
    assert seen[-1] == "ERROR" and "DONE" not in seen
    assert fsm.delivery_verified is False
    assert "DELIVERY_NOT_VERIFIED" in fsm.failure_reason


def test_docking_failure_retries_then_errors(monkeypatch):
    fsm, m = fsm_with(["SUCCEEDED"], monkeypatch, dock_ok=False)
    seen = drive(fsm)
    assert seen[-1] == "ERROR"
    assert "DOCK_PICK" in fsm.failure_reason
    assert "pick" not in m.started                    # the arm never moved


def test_arm_not_ready_at_start_is_an_error(monkeypatch):
    fsm, m = fsm_with(["FAILED:STOW: INVALID_JOINT_SENSOR"], monkeypatch)
    seen = drive(fsm)
    assert seen == ["IDLE", "ARM_STOW", "ERROR"]


def test_unreachable_parcel_is_rejected_before_the_arm_moves(monkeypatch):
    from arm_kinematics import PlanarPitchArm
    import decision_maker as dm
    fsm, m = fsm_with(["SUCCEEDED"], monkeypatch)
    fsm.kin = PlanarPitchArm.from_config()
    monkeypatch.setattr(dm, "Docker", lambda line, **kw: InstantDocker(True, 0.9))
    seen = drive(fsm)
    assert seen[-1] == "ERROR"
    assert "PARCEL_OUT_OF_REACH" in fsm.failure_reason
    assert "pick" not in m.started


def test_legacy_fsm_without_arm_is_unchanged():
    """No manipulator -> the original placeholder states (old worlds/tests)."""
    planner = lambda s, g: ([g], {"length_m": 1.0})                  # noqa: E731
    est = lambda d, p, s, a: (0.4, -0.3, 0.0, 0.4)                    # noqa: E731
    fsm = DecisionMaker(planner, est, log=lambda m: None)
    assert fsm.manip is None
    fsm.update((0, -0.3, 0), None, None, [], 0.0, 0.1)
    assert fsm.state == "FIND_PACKAGE"
