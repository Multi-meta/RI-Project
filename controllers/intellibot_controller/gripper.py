"""gripper.py — parallel gripper, latch and grasp verification (pure).

Mechanism (world built by tools/generate_world.py):
    * two fingers on LinearMotors closing along the robot y axis, position
      sensors (travel, 0 = fully open) and force TouchSensor pads;
    * a palm range-finder (DistanceSensor, generic ray) looking along the
      gripper axis;
    * a latch = Webots Connector pair: an ACTIVE connector on the palm, a
      PASSIVE connector on every vertical face of each parcel. lock() creates
      a real physical joint (the documented Webots mechanism for "an object
      equipped with a passive connector grasped by a robotic gripper").

Closing is FORCE-LIMITED: the fingers are commanded to full travel with only
GRIPPER_GRIP_FORCE available, so they stall on the parcel. "Closing finished"
= fingers stopped moving for GRIPPER_STALL_TIME (on the parcel, or at the end
of travel when nothing was grasped).

Grasp verification = sensor fusion (checklist 11.9, addendum A4.4):

    width_ok  : |finger width - parcel size| <= GRIPPER_WIDTH_TOL  (encoders)
    force_ok  : both pad forces >= GRIPPER_FORCE_MIN                (touch)
    palm_ok   : palm range <= GRIPPER_PALM_PRESENT_MAX              (range-finder)
    latched   : connector locked                                    (latch state)

    HOLDING   = width_ok AND palm_ok AND (force_ok OR latched)

Two independent geometric checks (width, palm range) must agree that a
parcel-sized object is between the fingers AND at the palm; force OR latch
must confirm it is actually held. A missed grasp shows as width << size,
a parcel left behind after lifting as palm range >> present threshold.
"""

import math
from dataclasses import dataclass, asdict

import config


@dataclass
class GraspAssessment:
    holding: bool
    state: str              # HOLDING | MISSED | NO_CONTACT | PARTIAL | UNKNOWN
    width: float
    force_left: float
    force_right: float
    palm_range: float
    latch_present: int
    latched: bool
    width_ok: bool
    force_ok: bool
    palm_ok: bool
    reason: str = ""

    def as_dict(self):
        return asdict(self)


class Gripper:
    def __init__(self, driver, log=print):
        self.driver = driver
        self.log = log
        self.mode = "IDLE"          # OPENING | CLOSING | IDLE
        self.status = "IDLE"        # MOVING | DONE | FAULT | IDLE
        self.fault = ""
        self._t_cmd = 0.0
        self._history = []          # [(t, left, right)] inside the stall window
        self._contact_since = None
        self.done_reason = ""

    # ------------------------------------------------------------ geometry
    @staticmethod
    def width_from_travel(left: float, right: float) -> float:
        """Inner distance between the finger pads (m)."""
        return 2.0 * config.GRIPPER_OPEN_HALF_WIDTH - left - right

    def width(self):
        left, right = self.driver.read_fingers()
        if not (math.isfinite(left) and math.isfinite(right)):
            return math.nan
        return self.width_from_travel(left, right)

    # ------------------------------------------------------------ commands
    def open(self, t):
        self.driver.command_fingers(0.0, force=config.GRIPPER_FINGER_MAX_FORCE)
        self._begin("OPENING", t)

    def close(self, t):
        self.driver.command_fingers(config.GRIPPER_FINGER_TRAVEL,
                                    force=config.GRIPPER_GRIP_FORCE)
        self._begin("CLOSING", t)

    def _begin(self, mode, t):
        self.mode = mode
        self.status = "MOVING"
        self.fault = ""
        self._t_cmd = t
        self._history = []
        self._contact_since = None
        self.done_reason = ""

    def update(self, t) -> str:
        """Track finger motion: DONE when open / stalled-closed, FAULT on timeout.

        Closing is finished when, over the last GRIPPER_STALL_TIME, neither
        finger moved more than GRIPPER_STALL_WINDOW_TOL (window test: a
        finger jittering by a few 0.1 mm against a parcel under a
        force-limited motor used to reset a per-step "still" test forever and
        the step only ended at the timeout), OR both pads report contact
        force for GRIPPER_CONTACT_DONE_TIME with the width stable.
        """
        if self.status != "MOVING":
            return self.status
        fingers = self.driver.read_fingers()
        if not all(math.isfinite(v) for v in fingers):
            if t - self._t_cmd > 1.0:
                return self._set_fault("INVALID_FINGER_SENSOR")
            return self.status
        if self.mode == "OPENING":
            if all(v <= config.GRIPPER_POS_TOL for v in fingers):
                self.status = "DONE"
                self.done_reason = "open"
        else:  # CLOSING
            self._history.append((t, float(fingers[0]), float(fingers[1])))
            # keep exactly one sample at/before the window start
            while (len(self._history) > 1
                   and t - self._history[1][0] >= config.GRIPPER_STALL_TIME):
                self._history.pop(0)
            span = t - self._history[0][0]
            moved = max(max(h[i] for h in self._history) - min(h[i] for h in self._history)
                        for i in (1, 2))
            if span >= config.GRIPPER_STALL_TIME and moved < config.GRIPPER_STALL_WINDOW_TOL:
                self.status = "DONE"
                self.done_reason = "stalled"
            else:
                try:
                    fl, fr = self.driver.read_finger_forces()
                except Exception:
                    fl = fr = math.nan
                contact = (math.isfinite(fl) and math.isfinite(fr)
                           and min(fl, fr) >= config.GRIPPER_FORCE_MIN)
                if contact:
                    if self._contact_since is None:
                        self._contact_since = t
                    recent = [h for h in self._history
                              if t - h[0] <= config.GRIPPER_CONTACT_DONE_TIME]
                    stable = max(max(h[i] for h in recent) - min(h[i] for h in recent)
                                 for i in (1, 2)) < config.GRIPPER_STALL_WINDOW_TOL
                    if t - self._contact_since >= config.GRIPPER_CONTACT_DONE_TIME and stable:
                        self.status = "DONE"
                        self.done_reason = "contact"
                else:
                    self._contact_since = None
        travel_time = config.GRIPPER_FINGER_TRAVEL / config.GRIPPER_FINGER_MAX_VEL
        if self.status == "MOVING" and t - self._t_cmd > travel_time + 3.0:
            return self._set_fault(f"{self.mode}_TIMEOUT (fingers={fingers[0]:.4f},"
                                   f"{fingers[1]:.4f} m)")
        return self.status

    def _set_fault(self, reason):
        self.status = "FAULT"
        self.fault = reason
        self.log(f"[GRIPPER] FAULT {reason}")
        return self.status

    def is_open(self) -> bool:
        w = self.width()
        return math.isfinite(w) and w >= 2 * config.GRIPPER_OPEN_HALF_WIDTH - 2 * config.GRIPPER_POS_TOL

    # --------------------------------------------------------------- latch
    def latch_present(self) -> int:
        return self.driver.latch_presence()

    def attach(self):
        """Lock the latch; only allowed when the parcel's connector is present."""
        if self.driver.latch_presence() != 1:
            return False, "LATCH_PEER_NOT_PRESENT"
        self.driver.latch_lock()
        return True, ""

    def release(self):
        self.driver.latch_unlock()

    def latched(self) -> bool:
        return self.driver.latch_is_locked()

    # -------------------------------------------------------- verification
    def assess(self, expected_size: float = None) -> GraspAssessment:
        """Fuse finger width, pad forces, palm range and latch state."""
        if expected_size is None:
            expected_size = config.PKG_SIZE
        w = self.width()
        fl, fr = self.driver.read_finger_forces()
        pr = self.driver.read_palm_range()
        present = self.driver.latch_presence()
        latched = self.driver.latch_is_locked()
        valid = all(math.isfinite(v) for v in (w, fl, fr, pr))
        width_ok = valid and abs(w - expected_size) <= config.GRIPPER_WIDTH_TOL
        force_ok = valid and min(fl, fr) >= config.GRIPPER_FORCE_MIN
        palm_ok = valid and pr <= config.GRIPPER_PALM_PRESENT_MAX
        holding = bool(width_ok and palm_ok and (force_ok or latched))
        if not valid:
            state, reason = "UNKNOWN", "invalid sensor reading"
        elif holding:
            state, reason = "HOLDING", ""
        elif w < expected_size - config.GRIPPER_WIDTH_TOL:
            state, reason = "MISSED", f"fingers closed to {w * 100:.1f} cm (< parcel)"
        elif not palm_ok and not width_ok:
            state, reason = "NO_CONTACT", "nothing between the fingers / at the palm"
        else:
            state = "PARTIAL"
            missing = [n for n, ok in (("width", width_ok), ("palm", palm_ok),
                                       ("force|latch", force_ok or latched)) if not ok]
            reason = "check failed: " + ", ".join(missing)
        return GraspAssessment(holding, state, w, fl, fr, pr, present, latched,
                               bool(width_ok), bool(force_ok), bool(palm_ok), reason)
