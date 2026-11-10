"""pick_place.py — manipulation sequences with a sensor check at every step.

The mission FSM (decision_maker.py) owns WHEN to manipulate; this module owns
HOW. Each operation is a short list of named steps; a step starts an action
(arm motion, gripper command, latch) and then polls a completion condition.
Every step has a timeout, and any failure stops the sequence with a reason —
the FSM then recovers or reports ERROR. Nothing is ever declared successful
without a sensor reading that confirms it.

Operations (robot-frame TCP targets come from shelving.py):

    pick              OPEN_GRIPPER -> PRE_GRASP -> DESCEND -> ADVANCE ->
                      CHECK_AT_PALM -> CLOSE_GRIPPER -> CHECK_GRASP -> LATCH ->
                      LIFT_CHECK
    verify_attachment SETTLE -> CHECK_ATTACHED -> TRANSIT_UP -> STOW ->
                      CHECK_CARRY
    place             CHECK_HOLDING -> PRE_PLACE -> INSERT -> LOWER ->
                      UNLATCH -> OPEN_GRIPPER
    verify_delivery   SETTLE -> RETRACT -> CHECK_RELEASED -> BACK_OUT -> STOW
    recover           UNLATCH/OPEN (unless keeping the parcel) -> RAISE -> STOW
    stow              STOW

Status: IDLE | RUNNING | SUCCEEDED | FAILED  (reason in .reason)
"""

import math

import config


class Step:
    """start(t) -> (ok, reason); poll(t) -> 'RUNNING' | 'DONE' | ('FAILED', reason)."""

    def __init__(self, name, start, poll, timeout=60.0):
        self.name = name
        self.start = start
        self.poll = poll
        self.timeout = timeout


class PickPlaceController:
    def __init__(self, arm, gripper, log=print):
        self.arm = arm
        self.gripper = gripper
        self.log = log
        self.status = "IDLE"
        self.operation = ""
        self.reason = ""
        self.steps = []
        self.i = 0
        self.t_step = 0.0
        self.history = []            # [(t, operation, step, result)]
        self.last_assessment = None
        self.last_check = {}

    # ----------------------------------------------------------- execution
    @property
    def step_name(self):
        return self.steps[self.i].name if 0 <= self.i < len(self.steps) else ""

    def _begin(self, operation, steps, t):
        self.operation = operation
        self.steps = steps
        self.i = 0
        self.status = "RUNNING"
        self.reason = ""
        self.log(f"[MANIP] {operation}: " + " -> ".join(s.name for s in steps))
        self._start_step(t)

    def _start_step(self, t):
        if self.i >= len(self.steps):
            self._succeed(t)
            return
        step = self.steps[self.i]
        self.t_step = t
        ok, reason = step.start(t)
        if not ok:
            self._fail(step.name, reason, t)

    def _fail(self, step, reason, t):
        self.status = "FAILED"
        self.reason = f"{step}: {reason}"
        self.history.append((round(t, 2), self.operation, step, "FAILED " + reason))
        self.log(f"[MANIP] {self.operation} FAILED at {step}: {reason}")

    def _succeed(self, t):
        self.status = "SUCCEEDED"
        self.history.append((round(t, 2), self.operation, "-", "SUCCEEDED"))
        self.log(f"[MANIP] {self.operation} SUCCEEDED")

    def update(self, t) -> str:
        """Advance the running operation by one control step."""
        if self.status != "RUNNING":
            return self.status
        self.arm.update(t)
        self.gripper.update(t)
        step = self.steps[self.i]
        res = step.poll(t)
        if res == "RUNNING":
            if t - self.t_step > step.timeout:
                self._fail(step.name, f"TIMEOUT after {step.timeout:.0f} s", t)
            return self.status
        if isinstance(res, tuple):
            self._fail(step.name, res[1], t)
            return self.status
        self.history.append((round(t, 2), self.operation, step.name, "DONE"))
        self.i += 1
        self._start_step(t)
        return self.status

    # ------------------------------------------------------- step factory
    def _arm_poll(self, t):
        st = self.arm.update(t)
        if st == "DONE":
            return "DONE"
        if st == "FAULT":
            return ("FAILED", self.arm.fault)
        return "RUNNING"

    def _line(self, name, target_fn, fail_prefix=""):
        """Straight TCP line; target_fn() -> (x, z) evaluated at step start."""
        def start(t):
            x, z = target_fn()
            ok, reason = self.arm.move_line(x, z, t, name=name)
            return ok, (fail_prefix + reason) if not ok else ""

        def poll(t):
            res = self._arm_poll(t)
            if isinstance(res, tuple) and fail_prefix:
                return ("FAILED", fail_prefix + res[1])
            return res
        return Step(name, start, poll)

    def _joint_to_tcp(self, name, x, z):
        def start(t):
            return self.arm.move_to_tcp(x, z, t, name=name)
        return Step(name, start, self._arm_poll)

    def _gripper(self, name, opening):
        def start(t):
            (self.gripper.open if opening else self.gripper.close)(t)
            return True, ""

        def poll(t):
            st = self.gripper.update(t)
            if st == "DONE":
                if opening and not self.gripper.is_open():
                    return ("FAILED", "GRIPPER_DID_NOT_OPEN")
                return "DONE"
            if st == "FAULT":
                return ("FAILED", self.gripper.fault)
            return "RUNNING"
        return Step(name, start, poll, timeout=10.0)

    def _wait(self, name, duration):
        t0 = {}

        def start(t):
            t0["t"] = t
            return True, ""

        def poll(t):
            return "DONE" if t - t0["t"] >= duration else "RUNNING"
        return Step(name, start, poll, timeout=duration + 5.0)

    def _check(self, name, fn):
        """Instant check: fn() -> (ok, reason, details)."""
        result = {}

        def start(t):
            ok, reason, details = fn()
            result.update(ok=ok, reason=reason)
            self.last_check[name] = details
            self.log(f"[MANIP] {name}: {'OK' if ok else 'FAILED'} {details}")
            return True, ""

        def poll(t):
            return "DONE" if result["ok"] else ("FAILED", result["reason"])
        return Step(name, start, poll, timeout=1.0)

    def _latch(self):
        def start(t):
            return self.gripper.attach()

        def poll(t):
            return "DONE" if self.gripper.latched() else "RUNNING"
        return Step("LATCH", start, poll, timeout=config.CONNECTOR_LOCK_TIMEOUT)

    def _unlatch(self):
        def start(t):
            self.gripper.release()
            return True, ""

        def poll(t):
            return "DONE" if not self.gripper.latched() else "RUNNING"
        return Step("UNLATCH", start, poll, timeout=config.CONNECTOR_LOCK_TIMEOUT)

    # ---------------------------------------------------------------- checks
    def _assess(self):
        a = self.gripper.assess()
        self.last_assessment = a
        return a

    def _check_holding(self, failure):
        def fn():
            a = self._assess()
            details = (f"width={a.width * 100:.1f}cm force=({a.force_left:.1f},"
                       f"{a.force_right:.1f})N palm={a.palm_range * 100:.1f}cm "
                       f"latched={a.latched} -> {a.state}")
            return a.holding, f"{failure} ({a.state}: {a.reason})", details
        return fn

    def _check_grasp(self):
        """Before latching: geometry must say 'parcel between the fingers'."""
        a = self._assess()
        details = (f"width={a.width * 100:.1f}cm force=({a.force_left:.1f},"
                   f"{a.force_right:.1f})N palm={a.palm_range * 100:.1f}cm "
                   f"latch_present={a.latch_present}")
        if a.width_ok and a.palm_ok:
            return True, "", details
        if a.state == "MISSED":
            return False, f"GRASP_MISSED ({a.reason})", details
        return False, f"GRASP_UNVERIFIED ({a.reason})", details

    def _check_at_palm(self):
        r = self.gripper.driver.read_palm_range()
        details = f"palm_range={r * 100:.1f}cm" if math.isfinite(r) else "palm_range=NaN"
        ok = math.isfinite(r) and r <= config.GRIPPER_PALM_PRESENT_MAX + 0.01
        return ok, f"PARCEL_NOT_AT_PALM ({details})", details

    def _check_released(self):
        r = self.gripper.driver.read_palm_range()
        expected = config.GRIPPER_PALM_GAP + config.ARM_RETRACT_DIST
        latched = self.gripper.latched()
        is_open = self.gripper.is_open()
        details = (f"palm_range={r * 100:.1f}cm expected={expected * 100:.1f}cm "
                   f"latched={latched} open={is_open}")
        if not math.isfinite(r):
            return False, "INVALID_PALM_RANGE", details
        if latched:
            return False, "LATCH_STILL_LOCKED", details
        if r < expected - config.ARM_VERIFY_RANGE_TOL:
            return False, "PARCEL_FOLLOWED_GRIPPER", details
        if r > expected + config.ARM_VERIFY_RANGE_TOL:
            return False, "PARCEL_NOT_IN_SLOT", details
        return True, "", details

    # ------------------------------------------------------------ operations
    def start_pick(self, t, grasp):
        """grasp: {'x', 'z'} parcel-centre TCP target in the robot frame."""
        xg, zg = float(grasp["x"]), float(grasp["z"])
        xp = xg - config.ARM_PREGRASP_BACKOFF
        steps = [
            self._gripper("OPEN_GRIPPER", opening=True),
            self._joint_to_tcp("PRE_GRASP", xp, zg + config.ARM_PREGRASP_RAISE),
            self._line("DESCEND", lambda: (xp, zg)),
            self._line("ADVANCE", lambda: (xg, zg)),
            self._check("CHECK_AT_PALM", self._check_at_palm),
            self._gripper("CLOSE_GRIPPER", opening=False),
            self._check("CHECK_GRASP", self._check_grasp),
            self._latch(),
            self._line("LIFT_CHECK", lambda: (xg, zg + config.ARM_LIFT_CHECK_DZ)),
        ]
        self._begin("pick", steps, t)

    def start_verify_attachment(self, t):
        def transit_target():
            x, z = self.arm.tcp()
            return x, max(z, config.ARM_SAFE_TRANSIT_Z)
        steps = [
            self._wait("SETTLE", 0.3),
            self._check("CHECK_ATTACHED", self._check_holding("ATTACHMENT_LOST")),
            self._line("TRANSIT_UP", transit_target),
            self._line("STOW", lambda: config.ARM_STOW_TCP),
            self._check("CHECK_CARRY", self._check_holding("ATTACHMENT_LOST_DURING_STOW")),
        ]
        self._begin("verify_attachment", steps, t)

    def start_place(self, t, place):
        """place: dict from shelving.place_tcp_for_slot (robot frame)."""
        steps = [
            self._check("CHECK_HOLDING", self._check_holding("PARCEL_LOST_IN_TRANSIT")),
            self._line("PRE_PLACE", lambda: (place["pre_x"], place["z_hover"])),
            self._line("INSERT", lambda: (place["x"], place["z_hover"]),
                       fail_prefix="SLOT_BLOCKED: "),
            self._line("LOWER", lambda: (place["x"], place["z_release"])),
            self._unlatch(),
            self._gripper("OPEN_GRIPPER", opening=True),
        ]
        self._begin("place", steps, t)

    def start_verify_delivery(self, t, place):
        steps = [
            self._wait("SETTLE", config.ARM_SETTLE_TIME),
            self._line("RETRACT", lambda: (place["retract_x"], place["z_release"])),
            self._check("CHECK_RELEASED", self._check_released),
            self._line("BACK_OUT", lambda: (place["pre_x"], place["z_release"])),
            self._line("STOW", lambda: config.ARM_STOW_TCP),
        ]
        self._begin("verify_delivery", steps, t)

    def start_recover(self, t, keep_parcel=False, retreat_x=None):
        """Bring the arm back to stow after a failure (base must be stopped).

        keep_parcel : True when a held parcel must stay latched (e.g. the
                      target slot was blocked) — otherwise unlatch + open.
        retreat_x   : TCP x to back off to horizontally before rising (e.g.
                      the pre-place x outside the rack). Default: pull back
                      by the pre-grasp back-off (fingers leave the parcel).
        """
        def retreat_target():
            tcp = self.arm.tcp()
            if tcp is None:
                return config.ARM_STOW_TCP
            x, z = tcp
            rx = retreat_x if retreat_x is not None else x - config.ARM_PREGRASP_BACKOFF
            return min(x, rx), z

        def raise_target():
            x, z = self.arm.tcp()
            return x, max(z, config.ARM_SAFE_TRANSIT_Z)
        steps = []
        if not keep_parcel:
            steps += [self._unlatch(), self._gripper("OPEN_GRIPPER", opening=True)]
        steps += [self._line("RETREAT", retreat_target),
                  self._line("RAISE", raise_target),
                  self._line("STOW", lambda: config.ARM_STOW_TCP)]
        self._begin("recover", steps, t)

    def start_stow(self, t):
        self._begin("stow", [self._joint_to_tcp("STOW", *config.ARM_STOW_TCP)], t)


def build_manipulator(driver, log=print):
    """Driver -> (PickPlaceController, PlanarPitchArm). One place to wire it."""
    from arm_controller import ArmController
    from arm_kinematics import PlanarPitchArm
    from gripper import Gripper
    kin = PlanarPitchArm.from_config()
    arm = ArmController(driver, kin, log=log)
    grip = Gripper(driver, log=log)
    return PickPlaceController(arm, grip, log=log), kin


def manip_log_row(manip, sim_time, state):
    """One row for logs/manip*.csv (logger.MANIP_COLUMNS)."""
    q = manip.arm.joints()
    tcp = manip.arm.kin.forward(q)["TCP"] if q is not None else (math.nan, math.nan)
    drv = manip.gripper.driver
    fl, fr = drv.read_finger_forces()
    r4 = lambda v: round(v, 4) if isinstance(v, float) and math.isfinite(v) else v  # noqa: E731
    return dict(t=round(sim_time, 3), state=state, operation=manip.operation,
                step=manip.step_name if manip.status == "RUNNING" else manip.status,
                q_shoulder=r4(q[0]) if q else "", q_elbow=r4(q[1]) if q else "",
                q_wrist=r4(q[2]) if q else "", tcp_x=r4(tcp[0]), tcp_z=r4(tcp[1]),
                finger_width=r4(manip.gripper.width()), force_left=r4(fl),
                force_right=r4(fr), palm_range=r4(drv.read_palm_range()),
                latch_present=drv.latch_presence(), latched=int(drv.latch_is_locked()),
                arm_status=manip.arm.status)
