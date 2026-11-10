"""arm_controller.py — joint-space and Cartesian motion of the arm (pure).

Responsibilities
    * keep every command inside the joint limits (clamp + IK limit check);
    * generate smooth motions: quintic joint trajectories (trajectory.py) and
      straight TCP lines resolved by IK every ARM_CART_STEP metres with a
      quintic time scaling (zero velocity/acceleration at both ends);
    * monitor execution with the joint PositionSensors: done = trajectory
      finished AND every joint within ARM_JOINT_TOL; FAULT = tracking error
      above ARM_TRACKING_ERROR_MAX for ARM_TRACKING_FAULT_TIME (arm blocked or
      in collision), invalid sensors, or settle timeout.

It talks to hardware only through a driver (arm_driver.WebotsArmDriver in
Webots, a fake in tests): read_joints() / command_joints(q).

Status values returned by update(): IDLE, MOVING, DONE, FAULT.
"""

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

import config
from arm_kinematics import PlanarPitchArm
from trajectory import QuinticTrajectory, quintic_profile


class _PathTrajectory:
    """Joint waypoints (from a Cartesian line) followed with quintic timing.

    The path is parametrised by a per-segment "time cost" — the time the
    slowest joint (at 80 % of its limit) or the TCP (at ARM_CART_SPEED) needs
    for that segment — so dense samples near a singular configuration get
    proportionally more time and no joint exceeds its velocity limit. The
    quintic s(tau) then scales the whole path (peak factor 1.875).
    """

    def __init__(self, qs: List[Tuple[float, float, float]], seg_costs, duration):
        self.qs = np.asarray(qs, dtype=float)
        c = np.concatenate([[0.0], np.cumsum(seg_costs)])
        self.u = c / c[-1] if c[-1] > 0 else np.linspace(0.0, 1.0, len(qs))
        self.T = max(float(duration), 0.05)

    def sample(self, t):
        s, _sd, _sdd = quintic_profile(t / self.T)
        i = int(np.clip(np.searchsorted(self.u, s, side="right") - 1, 0, len(self.qs) - 2))
        du = self.u[i + 1] - self.u[i]
        f = 0.0 if du <= 0 else (s - self.u[i]) / du
        return self.qs[i] * (1 - f) + self.qs[i + 1] * f

    @property
    def q1(self):
        return self.qs[-1]


class ArmController:
    def __init__(self, driver, kin: Optional[PlanarPitchArm] = None, log=print):
        self.driver = driver
        self.kin = kin if kin is not None else PlanarPitchArm.from_config()
        self.log = log
        self.traj = None
        self.t0 = 0.0
        self.goal = None
        self.status = "IDLE"
        self.fault = ""
        self._err_since = None
        self._invalid_since = None
        self.motion_name = ""
        self.max_tracking_error = 0.0
        self.settle_note = ""
        self._q_hist = []           # [(t, q)] after the trajectory ended
        self.last_diagnosis = []

    # ------------------------------------------------------------ queries
    def joints(self) -> Optional[Tuple[float, float, float]]:
        """Measured joint angles, or None while any sensor is invalid."""
        q = self.driver.read_joints()
        if q is None or len(q) != 3 or not all(math.isfinite(v) for v in q):
            return None
        return tuple(float(v) for v in q)

    def tcp(self) -> Optional[Tuple[float, float]]:
        q = self.joints()
        return None if q is None else self.kin.forward(q)["TCP"]

    def at(self, q_target, tol=None) -> bool:
        q = self.joints()
        if q is None:
            return False
        tol = config.ARM_JOINT_TOL if tol is None else tol
        return all(abs(a - b) <= tol for a, b in zip(q, q_target))

    # ------------------------------------------------------------ commands
    def _start(self, traj, goal, t, name):
        self.traj = traj
        self.goal = tuple(float(v) for v in goal)
        self.t0 = t
        self.status = "MOVING"
        self.fault = ""
        self._err_since = None
        self._invalid_since = None
        self.motion_name = name
        self.max_tracking_error = 0.0
        self.settle_note = ""
        self._q_hist = []
        return True, ""

    def move_joints(self, q_goal: Sequence[float], t: float, name="joint_move"):
        """Quintic joint-space motion from the MEASURED joints to q_goal."""
        if not self.kin.within_limits(q_goal):
            return False, "JOINT_LIMIT"
        q0 = self.joints()
        if q0 is None:
            return False, "INVALID_JOINT_SENSOR"
        vmax = min(config.ARM_JOINT_MAX_VEL) * 0.8
        traj = QuinticTrajectory(np.array(q0), np.array(q_goal),
                                 max_velocity=vmax,
                                 max_acceleration=config.ARM_JOINT_MAX_ACC)
        return self._start(traj, q_goal, t, name)

    def move_to_tcp(self, x: float, z: float, t: float, name="tcp_move", phi=0.0):
        """Joint-space motion to the IK solution of a TCP target (elbow up)."""
        q, reason = self.kin.solve(x, z, phi, elbow_up=True)
        if q is None:
            return False, reason
        return self.move_joints(q, t, name)

    def move_line(self, x: float, z: float, t: float, name="line_move",
                  phi=0.0, speed=None):
        """Straight TCP line from the current TCP to (x, z).

        Every ARM_CART_STEP the IK is solved (elbow up, limits checked); the
        resulting joint path must be continuous (< 0.25 rad between samples).
        Duration = max(length / speed, slowest joint at 80 % of its limit).
        """
        q0 = self.joints()
        if q0 is None:
            return False, "INVALID_JOINT_SENSOR"
        q_end, reason = self.kin.solve(x, z, phi, elbow_up=True)
        if q_end is None:
            return False, reason                       # target itself unreachable
        x0, z0 = self.kin.forward(q0)["TCP"]
        length = math.hypot(x - x0, z - z0)
        n = max(1, int(math.ceil(length / config.ARM_CART_STEP)))
        qs = [q0]
        f_prev = 0.0
        for i in range(1, n + 1):
            seg, reason = self._refine_segment(x0, z0, x, z, phi, f_prev, i / n, qs[-1])
            if seg is None:
                return False, reason
            qs.extend(seg)
            f_prev = i / n
        speed = config.ARM_CART_SPEED if speed is None else speed
        qa = np.asarray(qs)
        vlim = 0.8 * np.asarray(config.ARM_JOINT_MAX_VEL)
        tcps = np.array([self.kin.forward(q)["TCP"] for q in qs])
        seg_joint = np.max(np.abs(np.diff(qa, axis=0)) / vlim, axis=1)
        seg_tcp = np.linalg.norm(np.diff(tcps, axis=0), axis=1) / speed
        seg_cost = np.maximum(seg_joint, seg_tcp)
        duration = max(1.875 * float(seg_cost.sum()), 0.3)
        return self._start(_PathTrajectory(qs, seg_cost, duration), qs[-1], t, name)

    def _refine_segment(self, x0, z0, x1, z1, phi, fa, fb, q_prev, depth=0):
        """IK samples between path fractions fa..fb, bisected until every
        joint changes < 0.05 rad between samples (near the folded stow pose a
        1 cm TCP step can move the shoulder by ~0.2 rad). A jump that does
        not shrink with bisection is a branch flip -> IK_DISCONTINUITY."""
        q, reason = self.kin.solve(x0 + (x1 - x0) * fb, z0 + (z1 - z0) * fb, phi,
                                   elbow_up=True)
        if q is None:
            return None, reason
        jump = max(abs(a - b) for a, b in zip(q, q_prev))
        if jump <= 0.05:
            return [q], ""
        if depth >= 8:
            return (None, "IK_DISCONTINUITY") if jump > 0.25 else ([q], "")
        fm = 0.5 * (fa + fb)
        left, reason = self._refine_segment(x0, z0, x1, z1, phi, fa, fm, q_prev, depth + 1)
        if left is None:
            return None, reason
        right, reason = self._refine_segment(x0, z0, x1, z1, phi, fm, fb, left[-1], depth + 1)
        if right is None:
            return None, reason
        return left + right, ""

    def hold(self):
        """Stop where the arm is (used on faults)."""
        q = self.joints()
        if q is not None:
            self.driver.command_joints(self.kin.clamp(q))
        self.traj = None
        if self.status == "MOVING":
            self.status = "IDLE"

    # -------------------------------------------------------------- update
    def update(self, t: float) -> str:
        """Send the current setpoint; returns IDLE / MOVING / DONE / FAULT."""
        if self.traj is None or self.status in ("DONE", "FAULT", "IDLE"):
            return self.status
        tau = t - self.t0
        tau_c = min(tau, self.traj.T)
        if isinstance(self.traj, QuinticTrajectory):
            q_ref = self.traj.sample(tau_c)[0]
        else:
            q_ref = self.traj.sample(tau_c)
        q_cmd = self.kin.clamp(q_ref)
        self.driver.command_joints(q_cmd)
        q = self.joints()
        if q is None:
            if self._invalid_since is None:
                self._invalid_since = t
            if t - self._invalid_since > 0.3:
                return self._set_fault("INVALID_JOINT_SENSOR")
            return self.status
        self._invalid_since = None
        err = max(abs(a - b) for a, b in zip(q, q_cmd))
        self.max_tracking_error = max(self.max_tracking_error, err)
        if err > config.ARM_TRACKING_ERROR_MAX:
            if self._err_since is None:
                self._err_since = t
            if t - self._err_since > config.ARM_TRACKING_FAULT_TIME:
                j = int(np.argmax([abs(a - b) for a, b in zip(q, q_cmd)]))
                name = ("shoulder", "elbow", "wrist")[j]
                return self._set_fault(f"TRACKING_ERROR({name}, {err:.2f} rad)")
        else:
            self._err_since = None
        if tau >= self.traj.T:
            if self.at(self.goal):
                self.status = "DONE"
                return self.status
            # Under load (a latched parcel) or in light contact the position
            # loop can stop a little short of the goal. Accept that ONLY when
            # the arm has really stopped (no motion over ARM_SETTLE_STILL_TIME)
            # and the residual is inside ARM_JOINT_TOL_LOADED; the sensor
            # checks that follow every motion still verify the outcome.
            self._q_hist.append((t, q))
            while (len(self._q_hist) > 1
                   and t - self._q_hist[1][0] >= config.ARM_SETTLE_STILL_TIME):
                self._q_hist.pop(0)
            if t - self._q_hist[0][0] >= config.ARM_SETTLE_STILL_TIME:
                drift = max(max(h[1][j] for h in self._q_hist) - min(h[1][j] for h in self._q_hist)
                            for j in range(3))
                resid = max(abs(a - b) for a, b in zip(q, self.goal))
                if drift < config.ARM_SETTLE_STILL_TOL and resid <= config.ARM_JOINT_TOL_LOADED:
                    self.settle_note = f"settled {resid:.3f} rad from goal (stopped)"
                    self.log(f"[ARM] {self.motion_name}: {self.settle_note}")
                    self.status = "DONE"
                    return self.status
            if tau > self.traj.T + config.ARM_MOTION_TIMEOUT_MARGIN:
                return self._set_fault("SETTLE_TIMEOUT")
        return self.status

    def diagnose(self):
        """Per-joint table for fault reports: target, measured, error, torque,
        distance to the joint limits. Returns a list of dicts (also logged)."""
        q = self.driver.read_joints()
        torques = None
        if hasattr(self.driver, "read_torques"):
            try:
                torques = self.driver.read_torques()
            except Exception:
                torques = None
        rows = []
        names = ("shoulder", "elbow", "wrist")
        for j in range(3):
            meas = q[j] if q is not None and j < len(q) else math.nan
            tgt = self.goal[j] if self.goal is not None else math.nan
            lo, hi = config.ARM_JOINT_LIMITS_RAD[j]
            tq = torques[j] if torques is not None and j < len(torques) else math.nan
            rows.append(dict(joint=names[j], target=tgt, measured=meas,
                             error=(tgt - meas) if math.isfinite(meas) else math.nan,
                             torque=tq, max_torque=config.ARM_JOINT_MAX_TORQUE[j],
                             torque_saturated=(math.isfinite(tq) and
                                               abs(tq) >= 0.95 * config.ARM_JOINT_MAX_TORQUE[j]),
                             near_limit=(math.isfinite(meas) and
                                         min(meas - lo, hi - meas) < 0.05)))
        self.last_diagnosis = rows
        for r in rows:
            self.log(f"[ARM]   {r['joint']:8s} target={r['target']:+.3f} "
                     f"measured={r['measured']:+.3f} err={r['error']:+.3f} rad "
                     f"torque={r['torque']:.2f}/{r['max_torque']:.0f} Nm"
                     f"{' SATURATED' if r['torque_saturated'] else ''}"
                     f"{' NEAR_LIMIT' if r['near_limit'] else ''}")
        return rows

    def _set_fault(self, reason):
        self.status = "FAULT"
        self.fault = f"{self.motion_name}: {reason}"
        self.log(f"[ARM] FAULT {self.fault}")
        try:
            self.diagnose()
        except Exception as exc:                      # diagnostics must never crash
            self.log(f"[ARM]   diagnosis unavailable: {exc}")
        self.hold()
        self.status = "FAULT"
        return self.status
