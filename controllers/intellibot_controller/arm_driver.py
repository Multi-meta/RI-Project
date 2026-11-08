"""arm_driver.py — Webots device adapter for the arm, gripper and latch.

This is the ONLY place the manipulator touches the Webots API. It holds no
logic: it converts between Webots devices and plain numbers so that
arm_controller.py, gripper.py and pick_place.py stay pure and are tested with
a fake driver exposing the same methods (tests/fakes_arm.py,
tools/offline_mission_sim.py).

Driver interface (all units SI, joint angles in config convention):
    available / missing             all devices found? names of missing ones
    read_joints()        -> [q1, q2, q3]   (NaN while a sensor is not valid)
    command_joints(q)                       position setpoints
    read_fingers()       -> (left, right)   finger travel, 0 = fully open
    command_fingers(pos, force=None)        same travel for both fingers
    read_finger_forces() -> (left, right)   N, contact force on the pads
    read_palm_range()    -> metres          palm range-finder (NaN if invalid)
    latch_presence()     -> 1 / 0 / -1      Connector.getPresence()
    latch_lock(), latch_unlock(), latch_is_locked()

Webots notes (docs/webots_findings.md, Webots reference manual):
    * PositionSensor / TouchSensor / DistanceSensor values are only valid
      after the first sampling period -> NaN until then (handled by callers).
    * Connector.getPresence() is "active only after the first sampling
      period"; isLocked() reports the latch STATE, not a physical link —
      the attachment is therefore verified with independent sensors
      (finger width, finger force, palm range) in gripper.py.
"""

import math

import config


class WebotsArmDriver:
    """Wraps the manipulator devices of the mission robot."""

    def __init__(self, robot, timestep):
        self.robot = robot
        self.dt = int(timestep)
        self.missing = []
        get = self._get
        self.joint_motors = [get(n) for n in config.ARM_MOTOR_NAMES]
        self.joint_sensors = [get(n) for n in config.ARM_SENSOR_NAMES]
        self.finger_motors = [get(n) for n in config.GRIPPER_FINGER_MOTOR_NAMES]
        self.finger_sensors = [get(n) for n in config.GRIPPER_FINGER_SENSOR_NAMES]
        self.finger_touch = [get(n) for n in config.GRIPPER_FINGER_TOUCH_NAMES]
        self.palm_range = get(config.GRIPPER_PALM_RANGE_NAME)
        self.latch = get(config.GRIPPER_CONNECTOR_NAME)
        self.available = not self.missing
        if not self.available:
            return
        for m, vmax in zip(self.joint_motors, config.ARM_JOINT_MAX_VEL):
            m.setVelocity(vmax)                      # speed used by position control
        for m in self.finger_motors:
            m.setVelocity(config.GRIPPER_FINGER_MAX_VEL)
            m.setAvailableForce(config.GRIPPER_GRIP_FORCE)
        for s in self.joint_sensors + self.finger_sensors + self.finger_touch:
            s.enable(self.dt)
        self.palm_range.enable(self.dt)
        self.latch.enablePresence(self.dt)

    def _get(self, name):
        """robot.getDevice that records missing names instead of crashing."""
        try:
            dev = self.robot.getDevice(name)
        except Exception:
            dev = None
        if dev is None:
            self.missing.append(name)
        return dev

    # ---------------------------------------------------------------- joints
    @staticmethod
    def _value(sensor):
        try:
            v = float(sensor.getValue())
        except Exception:
            return math.nan
        return v if math.isfinite(v) else math.nan

    def read_joints(self):
        return [self._value(s) for s in self.joint_sensors]

    def command_joints(self, q):
        for m, v in zip(self.joint_motors, q):
            m.setPosition(float(v))

    # --------------------------------------------------------------- fingers
    def read_fingers(self):
        return tuple(self._value(s) for s in self.finger_sensors)

    def command_fingers(self, pos, force=None):
        for m in self.finger_motors:
            if force is not None:
                m.setAvailableForce(float(force))
            m.setPosition(float(pos))

    def read_finger_forces(self):
        return tuple(abs(self._value(s)) for s in self.finger_touch)

    def read_palm_range(self):
        return self._value(self.palm_range)

    # ----------------------------------------------------------------- latch
    def latch_presence(self):
        try:
            return int(self.latch.getPresence())
        except Exception:
            return 0

    def latch_lock(self):
        self.latch.lock()

    def latch_unlock(self):
        self.latch.unlock()

    def latch_is_locked(self):
        try:
            return bool(self.latch.isLocked())
        except Exception:
            return False


class ParcelAuditor:
    """Supervisor ground truth for EVALUATION ONLY (never used for decisions).

    Reads the parcel Solid's world position so the mission report can state
    independently whether the parcel really ended up on the shelf.
    """

    def __init__(self, supervisor):
        self.sup = supervisor

    def position(self, def_name):
        try:
            node = self.sup.getFromDef(def_name)
        except Exception:
            node = None
        if node is None:
            return None
        p = node.getPosition()
        return (p[0], p[1], p[2]) if p is not None else None
