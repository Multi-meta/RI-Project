"""Fake manipulator hardware for unit tests (implements the arm_driver API).

Joints track their commands with a rate limit (like Webots position control
with maxVelocity); fingers close until ``object_width`` (if an object is
between them) and then report ``grip_force``; palm range, latch presence and
lock state are plain attributes the test sets. ``freeze_joint`` makes a joint
stop following commands (= the arm is blocked by an obstacle).
"""

import math

import numpy as np

import config
from arm_kinematics import PlanarPitchArm


class FakeArmDriver:
    def __init__(self, q0=None):
        kin = PlanarPitchArm.from_config()
        self.q = np.array(q0 if q0 is not None else kin.inverse(*config.ARM_STOW_TCP, 0.0))
        self.q_cmd = self.q.copy()
        self.fingers = np.zeros(2)
        self.finger_cmd = 0.0
        self.object_width = None        # width of an object between the fingers
        self.grip_force = 10.0
        self.palm_range = config.GRIPPER_PALM_RANGE_MAX
        self.presence = 0
        self.locked = False
        self.lock_works = True          # False: lock() does not change state
        self.unlock_works = True
        self.freeze_joint = None        # index of a joint that no longer moves
        self.invalid_joints = False
        self.available, self.missing = True, []

    def step(self, dt):
        vmax = np.array(config.ARM_JOINT_MAX_VEL)
        dq = np.clip(self.q_cmd - self.q, -vmax * dt, vmax * dt)
        if self.freeze_joint is not None:
            dq[self.freeze_joint] = 0.0
        self.q = self.q + dq
        stop = config.GRIPPER_FINGER_TRAVEL
        if self.object_width is not None:
            stop = config.GRIPPER_OPEN_HALF_WIDTH - self.object_width / 2.0
        target = min(self.finger_cmd, stop)
        s = config.GRIPPER_FINGER_MAX_VEL * dt
        self.fingers = self.fingers + np.clip(target - self.fingers, -s, s)

    # --- driver API
    def read_joints(self):
        return [math.nan] * 3 if self.invalid_joints else list(self.q)

    def command_joints(self, q):
        self.q_cmd = np.array(q, dtype=float)

    def read_fingers(self):
        return float(self.fingers[0]), float(self.fingers[1])

    def command_fingers(self, pos, force=None):
        self.finger_cmd = float(pos)

    def read_finger_forces(self):
        touching = (self.object_width is not None and self.finger_cmd > 0 and
                    np.all(self.fingers >= config.GRIPPER_OPEN_HALF_WIDTH
                           - self.object_width / 2.0 - 1e-4))
        f = self.grip_force if touching else 0.0
        return f, f

    def read_palm_range(self):
        return self.palm_range

    def latch_presence(self):
        return self.presence

    def latch_lock(self):
        if self.lock_works:
            self.locked = True

    def latch_unlock(self):
        if self.unlock_works:
            self.locked = False

    def latch_is_locked(self):
        return self.locked


def run(controller_update, driver, t0=0.0, dt=0.016, max_t=60.0, done=None):
    """Step a controller + fake driver until ``done(status)`` or max_t."""
    t = t0
    status = None
    while t < t0 + max_t:
        status = controller_update(t)
        if done is not None and done(status):
            break
        driver.step(dt)
        t += dt
    return status, t
