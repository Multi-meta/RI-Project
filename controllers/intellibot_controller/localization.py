"""localization.py — the pose estimate the robot actually drives with.

Wraps ekf_localization.EKFLocalizer with everything the controller needs
around it (pure Python, no Webots, unit-tested in tests/test_localization.py):

* deferred initialisation from the first VALID GPS + IMU sample (Webots
  sensors are NaN before their first sampling period — the old code seeded
  the EKF from a fallback constant instead of the real start pose);
* prediction from SIGNED encoder increments (kinematics.Odometry.last_ds /
  last_dtheta). The old wiring used ``hypot(dx, dy)`` of two odometry poses
  (always >= 0, so reversing predicted FORWARD motion) and an unwrapped
  ``dtheta`` (a +-2 pi jump at the +-pi seam);
* motion-dependent process noise Q = Q_base + diag(a1 ds^2, a1 ds^2,
  a2 dtheta^2 + a3 ds^2) instead of a constant 2 cm per control step;
* sensor health: GPS / IMU trust in [0, 1] from staleness, invalid samples
  and the innovation-gate rejection rate -> R / trust (R inflation);
* divergence ("kidnapped robot") recovery: the chi^2 gate rejects GPS fixes
  that disagree with the prediction. After a collision or a manual
  displacement in Webots the wheels never saw the motion, so EVERY later fix
  was rejected and the robot kept planning from where it USED to be. Now, if
  the gate rejects GPS for EKF_KIDNAP_TIME while consecutive fixes agree with
  each other (a real displacement, not a glitch), the filter re-initialises
  on GPS + IMU and reports a pose jump so navigation replans from the real
  position;
* GPS dropout: prediction only (dead reckoning), covariance grows, status
  DEAD_RECKONING (the speed governor caps the speed);
* ablation modes for experiments: "ekf" (default), "odom" (dead reckoning
  only), "gps" (raw GPS + IMU, no filter).

Ground truth (Supervisor) is NEVER an input here; the controller only logs it
next to the estimate for RMSE / NEES evaluation.
"""

import math
from collections import deque

import numpy as np

import config
from ekf_localization import EKFLocalizer
from kinematics import wrap_to_pi


class _Health:
    """Trust of one sensor from staleness, validity and gate rejections."""

    def __init__(self, name, stale_time, window):
        self.name = name
        self.stale_time = stale_time
        self.last_valid_t = None
        self.results = deque(maxlen=window)    # True = accepted by the gate
        self.status = "UNKNOWN"
        self.trust = 0.0

    def observe(self, t, valid, accepted=None):
        if valid:
            self.last_valid_t = t
            if accepted is not None:
                self.results.append(bool(accepted))
        stale = self.last_valid_t is None or t - self.last_valid_t > self.stale_time
        if stale:
            self.status, self.trust = "FAILED", 0.0
            return
        if len(self.results) >= 5:
            rate = sum(self.results) / len(self.results)
        else:
            rate = 1.0
        if rate >= 0.8:
            self.status, self.trust = "OK", 1.0
        elif rate >= 0.4:
            self.status, self.trust = "DEGRADED", max(0.2, rate)
        else:
            self.status, self.trust = "DEGRADED", 0.2

    def r_inflation(self):
        return 1e6 if self.trust <= 0.0 else 1.0 / self.trust


class LocalizationManager:
    MODES = ("ekf", "odom", "gps")

    def __init__(self, mode=None, log=print):
        mode = (mode or getattr(config, "LOCALIZATION_MODE", "ekf")).lower()
        if mode not in self.MODES:
            raise ValueError(f"unknown localization mode {mode!r}")
        self.mode = mode
        self.log = log
        self.ekf = None
        self.initialized = False
        self.pose = None
        self.status = "UNINITIALIZED"
        self.gps_health = _Health("gps", config.LOC_GPS_STALE_TIME, 30)
        self.imu_health = _Health("imu", config.LOC_IMU_STALE_TIME, 30)
        self._reject_since = None
        self._reject_fixes = []
        self._imu_reject = []           # [(t, yaw)] while the IMU is gated out
        self.heading_reanchors = 0
        self.relocalizations = 0
        self.pose_jump = None           # (dx, dy, dtheta) of the last relocalisation, until consumed
        self.n_gps_accepted = 0
        self.n_gps_rejected = 0
        self.last_nis = math.nan
        self._last_log_t = -1e9

    # --------------------------------------------------------------- helpers
    @staticmethod
    def _valid_xy(z):
        return z is not None and len(z) >= 2 and all(math.isfinite(v) for v in z[:2])

    @staticmethod
    def _valid(v):
        return v is not None and math.isfinite(v)

    def _init(self, x, y, th, t):
        p0 = np.diag(config.LOC_P0)
        self.ekf = EKFLocalizer(np.array([x, y, th], dtype=float), p0,
                                np.diag(config.EKF_Q_BASE))
        self.ekf.verbose = False
        self.initialized = True
        self.pose = (x, y, th)
        self.status = "OK"
        self.log(f"[LOC] [t={t:.2f}s] initialised ({self.mode}) at "
                 f"({x:.2f}, {y:.2f}, {math.degrees(th):.0f} deg)")

    def consume_pose_jump(self):
        """Return and clear the last relocalisation jump (or None)."""
        j, self.pose_jump = self.pose_jump, None
        return j

    def covariance(self):
        return None if self.ekf is None else self.ekf.get_covariance()

    # ------------------------------------------------------------------ step
    def step(self, t, odom_inc=None, gps_xy=None, imu_yaw=None):
        """One control step.

        odom_inc : (ds, dtheta) SIGNED encoder increments, or None
        gps_xy   : (x, y) GPS fix or None / NaN (dropout)
        imu_yaw  : yaw (rad) or None / NaN
        Returns the pose (x, y, theta) used by navigation, or None before the
        first valid GPS + IMU sample.
        """
        gps_ok = self._valid_xy(gps_xy)
        imu_ok = self._valid(imu_yaw)
        if not self.initialized:
            self.gps_health.observe(t, gps_ok)
            self.imu_health.observe(t, imu_ok)
            if gps_ok and imu_ok:
                self._init(float(gps_xy[0]), float(gps_xy[1]), float(imu_yaw), t)
            return self.pose

        # ---- prediction (all modes except raw GPS)
        if odom_inc is not None and all(math.isfinite(v) for v in odom_inc):
            ds, dth = float(odom_inc[0]), wrap_to_pi(float(odom_inc[1]))
        else:
            ds, dth = 0.0, 0.0
        q = np.array(config.EKF_Q_BASE, dtype=float)
        a1, a2, a3 = config.EKF_MOTION_NOISE
        q += np.array([a1 * ds * ds, a1 * ds * ds, a2 * dth * dth + a3 * ds * ds])
        self.ekf.Q = np.diag(q)
        if self.mode != "gps":
            self.ekf.predict(ds, dth)

        if self.mode == "odom":
            self.gps_health.observe(t, gps_ok)
            self.imu_health.observe(t, imu_ok)
            self.pose = tuple(float(v) for v in self.ekf.get_pose())
            self.status = "DEAD_RECKONING_ONLY"
            return self.pose

        if self.mode == "gps":
            self.gps_health.observe(t, gps_ok, True if gps_ok else None)
            self.imu_health.observe(t, imu_ok, True if imu_ok else None)
            x, y, th = self.pose
            if gps_ok:
                x, y = float(gps_xy[0]), float(gps_xy[1])
            if imu_ok:
                th = float(imu_yaw)
            self.ekf.x = np.array([x, y, th])
            self.pose = (x, y, th)
            self.status = "OK" if gps_ok and imu_ok else "DEGRADED"
            return self.pose

        # ---- EKF corrections with health-scaled R
        if imu_ok:
            r_imu = config.EKF_R_IMU * self.imu_health.r_inflation()
            acc = self.ekf.update_imu(float(imu_yaw), r_imu,
                                      gate_threshold=config.EKF_GATE_IMU)
            self.imu_health.observe(t, True, acc)
            if acc:
                self._imu_reject = []
            else:
                self._on_imu_rejected(t, float(imu_yaw))
        else:
            self.imu_health.observe(t, False)
        if gps_ok:
            z = np.array([float(gps_xy[0]), float(gps_xy[1])])
            r_gps = np.diag(config.EKF_R_GPS) * self.gps_health.r_inflation()
            acc = self.ekf.update_gps(z, r_gps, gate_threshold=config.EKF_GATE_GPS)
            self.last_nis = self.ekf.nis_history[-1] if self.ekf.nis_history else math.nan
            if len(self.ekf.nis_history) > 500:
                del self.ekf.nis_history[:-100]
                del self.ekf.innovation_history[:-100]
            self.gps_health.observe(t, True, acc)
            if acc:
                self.n_gps_accepted += 1
                self._reject_since = None
                self._reject_fixes = []
            else:
                self.n_gps_rejected += 1
                self._on_gps_rejected(t, z, imu_yaw if imu_ok else None)
        else:
            self.gps_health.observe(t, False)

        self.pose = tuple(float(v) for v in self.ekf.get_pose())
        if not gps_ok and self.gps_health.status == "FAILED":
            self.status = "DEAD_RECKONING"
        elif self._reject_since is not None:
            self.status = "GPS_REJECTED"
        elif self.gps_health.status != "OK" or self.imu_health.status != "OK":
            self.status = "DEGRADED"
        else:
            self.status = "OK"
        return self.pose

    def _on_imu_rejected(self, t, yaw):
        """Heading re-anchor: consistent IMU yaws the gate keeps rejecting mean
        the robot really turned without its wheels seeing it (push, collision)."""
        self._imu_reject.append((t, yaw))
        if t - self._imu_reject[0][0] < config.EKF_IMU_REANCHOR_TIME:
            return
        yaws = np.array([y for _, y in self._imu_reject])
        ref = yaws[-1]
        spread = float(np.max(np.abs(np.arctan2(np.sin(yaws - ref), np.cos(yaws - ref)))))
        if spread > config.EKF_IMU_REANCHOR_SPREAD:
            self._imu_reject = self._imu_reject[-1:]     # not consistent: keep waiting
            return
        old = float(self.ekf.x[2])
        self.ekf.x[2] = ref
        self.ekf.P[2, 2] = max(self.ekf.P[2, 2], config.EKF_R_IMU)
        self.ekf.P[0, 2] = self.ekf.P[2, 0] = self.ekf.P[1, 2] = self.ekf.P[2, 1] = 0.0
        self.heading_reanchors += 1
        self._imu_reject = []
        self.log(f"[LOC] [t={t:.2f}s] heading RE-ANCHORED to the IMU after "
                 f"{config.EKF_IMU_REANCHOR_TIME:.1f} s of consistent rejected yaws: "
                 f"{math.degrees(wrap_to_pi(ref - old)):+.1f} deg (odometry heading disagreed "
                 f"with the IMU: wheel slip, a push/collision — or an IMU bias step)")

    def _on_gps_rejected(self, t, z, yaw):
        if self._reject_since is None:
            self._reject_since = t
            self._reject_fixes = []
        self._reject_fixes.append((t, z.copy()))
        self._reject_fixes = [(ti, zi) for ti, zi in self._reject_fixes
                              if t - ti <= config.EKF_KIDNAP_TIME + 0.5]
        if t - self._reject_since < config.EKF_KIDNAP_TIME:
            if t - self._last_log_t > 1.0:
                self._last_log_t = t
                self.log(f"[LOC] [t={t:.2f}s] GPS fix rejected by the gate "
                         f"(NIS={self.last_nis:.1f}) — holding the prediction")
            return
        # consistent fixes? (a glitching GPS jumps around, a displaced robot
        # produces fixes that agree with each other and with a slow motion)
        pts = np.array([zi for _, zi in self._reject_fixes])
        spread = float(np.max(np.linalg.norm(pts - pts.mean(axis=0), axis=1)))
        if spread > config.EKF_KIDNAP_MAX_SPREAD:
            self._reject_since = t            # keep waiting for a consistent run
            self._reject_fixes = [(t, z.copy())]
            return
        old = self.ekf.get_pose()
        th = float(yaw) if yaw is not None and math.isfinite(yaw) else float(old[2])
        self.ekf.x = np.array([float(z[0]), float(z[1]), th])
        self.ekf.P = np.diag(config.LOC_P0)
        jump = (float(z[0] - old[0]), float(z[1] - old[1]), float(wrap_to_pi(th - old[2])))
        self.pose_jump = jump
        self.relocalizations += 1
        self._reject_since = None
        self._reject_fixes = []
        self.log(f"[LOC] [t={t:.2f}s] RELOCALISED on GPS+IMU after "
                 f"{config.EKF_KIDNAP_TIME:.1f} s of consistent rejected fixes: "
                 f"jump ({jump[0]:+.2f}, {jump[1]:+.2f}) m, "
                 f"{math.degrees(jump[2]):+.0f} deg (robot displaced — collision "
                 f"or manual move)")

    # ---------------------------------------------------------------- status
    def summary(self):
        P = self.covariance()
        return dict(mode=self.mode, status=self.status,
                    gps=self.gps_health.status, gps_trust=round(self.gps_health.trust, 2),
                    imu=self.imu_health.status, imu_trust=round(self.imu_health.trust, 2),
                    cov_trace=(round(float(np.trace(P)), 6) if P is not None else None),
                    sigma_xy=(round(float(math.sqrt(max(P[0, 0], P[1, 1]))), 4)
                              if P is not None else None),
                    last_nis=(round(self.last_nis, 2) if math.isfinite(self.last_nis) else None),
                    gps_accepted=self.n_gps_accepted, gps_rejected=self.n_gps_rejected,
                    relocalizations=self.relocalizations,
                    heading_reanchors=self.heading_reanchors)

    def healthy(self):
        """True when the pose is good enough to map obstacles / drive at speed."""
        if not self.initialized:
            return False
        P = self.covariance()
        sig = math.sqrt(max(P[0, 0], P[1, 1])) if P is not None else 0.0
        return self.status in ("OK", "DEGRADED") and sig <= config.LOC_SIGMA_HEALTHY
