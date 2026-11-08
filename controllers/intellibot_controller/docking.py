"""docking.py — fine positioning of the base before the arm moves (pure).

The A* + waypoint follower brings the robot to within ~0.1-0.4 m of a target;
the arm (no lateral joint) needs the base on the docking line to within
+-1.5 cm and +-1.5 deg, at a measured distance from the target face. Docking
closes that gap with three simple, explainable controllers:

    TURN_TO_TARGET  rotate in place to face the target (parcel only)
    REFINE          fit the parcel's LiDAR outline -> centre, yaw, dock line
    TO_PREDOCK      point-stabilise to a pre-dock point on the line
    FACE            rotate in place to the line heading
                    (parcel: REFINE once more from this square-on view,
                    then TO_PREDOCK/FACE again — coarse-to-fine)
    CREEP           drive straight in; speed from the LiDAR range error,
                    small heading correction from the lateral error
    SETTLE          hold still until the pose stops changing
    DONE / FAILED   (reason in .reason)

Sensors: pose (EKF/GPS+IMU) for heading and lateral error, LiDAR for the
range to the face — the most accurate RELATIVE measurement on the robot.

Pure logic: update(pose, scan, angles, t) -> (v, w). Tested with a simulated
unicycle and a ray-cast LiDAR (tests/test_docking.py).
"""

import math
from typing import Optional

import numpy as np

import config
from kinematics import wrap_to_pi
from mapping import front_min_range
from shelving import DockLine


# ---------------------------------------------------------------- measurements
def face_range(scan, angles, window: Optional[float] = None) -> Optional[float]:
    """Median LiDAR range in +-window around straight ahead (None if no data)."""
    if scan is None or angles is None:
        return None
    if window is None:
        window = config.ARM_DOCK_RANGE_WINDOW
    scan = np.asarray(scan, dtype=float)
    angles = np.asarray(angles, dtype=float)
    mask = (np.abs(angles) <= window) & np.isfinite(scan)
    if not mask.any():
        return None
    return float(np.median(scan[mask]))


def _unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else v


def refine_parcel_from_scan(pose, scan, angles, est_xy, size: Optional[float] = None,
                            preferred_heading: Optional[float] = None
                            ) -> Optional[DockLine]:
    """Estimate the parcel's centre and yaw from the LiDAR, return a dock line.

    1. bearing/range of the estimated parcel centre in the robot frame;
    2. seed = the closest LiDAR return within +-ARM_DOCK_REFINE_WINDOW of that
       bearing (a point on the parcel); grow a contiguous segment while
       consecutive ranges differ by less than ARM_DOCK_FACE_JUMP (stops at
       the parcel's silhouette edge, separates it from neighbours / the wall);
    3. corner test: if a point lies > 1.5 cm from the chord of the segment,
       the robot sees TWO faces meeting at that corner;
         - two faces : centre = corner + (t_a + t_b) * size/2  (t = unit
                       vectors along each face away from the corner)
         - one face  : centre = face midpoint + inward normal * size/2
       box yaw = direction of the longest visible face (PCA);
    4. the four faces of the box give four possible approach headings
       (inward normals); the one closest to ``preferred_heading`` (the
       direction the planner approached from) is used, so the robot grasps
       the face that is open to the aisle, not one facing a neighbour.

    Returns None if no plausible parcel is found or the refined centre moved
    more than ARM_DOCK_MAX_REFINE_SHIFT from the estimate.
    """
    if scan is None or angles is None:
        return None
    if size is None:
        size = config.PKG_SIZE
    x, y, th = pose
    robot = np.array([x, y])
    scan = np.asarray(scan, dtype=float)
    angles = np.asarray(angles, dtype=float)
    ex, ey = est_xy
    r_est = math.hypot(ex - x, ey - y)
    phi_est = wrap_to_pi(math.atan2(ey - y, ex - x) - th)
    order = np.argsort(angles)
    a_sorted = angles[order]
    r_sorted = scan[order]
    dphi = np.abs([wrap_to_pi(a - phi_est) for a in a_sorted])
    near = (np.isfinite(r_sorted) & (r_sorted < r_est + size)
            & (dphi <= config.ARM_DOCK_REFINE_WINDOW))
    if not near.any():
        return None
    cand = np.where(near)[0]
    seed = cand[np.argmin(r_sorted[cand])]
    jump = config.ARM_DOCK_FACE_JUMP
    n = len(a_sorted)
    lo = hi = seed
    while True:
        j = (lo - 1) % n
        if j == hi or not np.isfinite(r_sorted[j]) or abs(r_sorted[j] - r_sorted[lo]) > jump:
            break
        lo = j
    while True:
        j = (hi + 1) % n
        if j == lo or not np.isfinite(r_sorted[j]) or abs(r_sorted[j] - r_sorted[hi]) > jump:
            break
        hi = j
    idx = [(lo + k) % n for k in range(((hi - lo) % n) + 1)]
    if len(idx) < 3:
        return None
    pts = np.array([(x + r_sorted[i] * math.cos(th + a_sorted[i]),
                     y + r_sorted[i] * math.sin(th + a_sorted[i])) for i in idx])
    span = float(np.linalg.norm(pts[-1] - pts[0]))
    if not (0.06 <= span <= size * math.sqrt(2) * 1.15):
        return None

    # ---- corner test (distance of every point to the end-to-end chord)
    chord = _unit(pts[-1] - pts[0])
    normal = np.array([-chord[1], chord[0]])
    dist = np.abs((pts - pts[0]) @ normal)
    k = int(np.argmax(dist))
    if dist[k] > 0.015 and 3 <= k <= len(pts) - 4:
        corner = pts[k]
        t_a = _unit(pts[0] - corner)
        t_b = _unit(pts[-1] - corner)
        center = corner + (t_a + t_b) * size / 2.0
        face_dir = t_a if np.linalg.norm(pts[0] - corner) >= np.linalg.norm(pts[-1] - corner) else t_b
    else:
        c = pts.mean(axis=0)
        cov = np.cov((pts - c).T)
        evals, evecs = np.linalg.eigh(cov)
        face_dir = evecs[:, np.argmax(evals)]
        nrm = np.array([-face_dir[1], face_dir[0]])
        if np.dot(nrm, c - robot) < 0:              # inward = away from the robot
            nrm = -nrm
        center = c + nrm * size / 2.0
    if math.hypot(center[0] - ex, center[1] - ey) > config.ARM_DOCK_MAX_REFINE_SHIFT:
        return None

    # ---- choose the grasp face: 4 candidate inward normals of the box
    base = math.atan2(face_dir[1], face_dir[0]) + math.pi / 2.0
    if preferred_heading is None:
        preferred_heading = math.atan2(center[1] - y, center[0] - x)
    heading = min((wrap_to_pi(base + k * math.pi / 2.0) for k in range(4)),
                  key=lambda h: abs(wrap_to_pi(h - preferred_heading)))
    u = np.array([math.cos(heading), math.sin(heading)])
    face = center - u * size / 2.0
    return DockLine(face_point=(float(face[0]), float(face[1])), heading=heading,
                    standoff=config.ARM_DOCK_PICK_STANDOFF)


# -------------------------------------------------------------------- docker
class Docker:
    """Docking state machine. Create one per docking attempt."""

    def __init__(self, line: Optional[DockLine], *, parcel_estimate=None,
                 preferred_heading=None, log=print, name="dock"):
        """line: known dock line (rack), or None when ``parcel_estimate``
        (world x, y of a floor parcel) must first be refined from the LiDAR."""
        if line is None and parcel_estimate is None:
            raise ValueError("need a dock line or a parcel estimate")
        self.line = line
        self.parcel_estimate = parcel_estimate
        self.preferred_heading = preferred_heading
        self.log = log
        self.name = name
        self.phase = "TURN_TO_TARGET" if line is None else "TO_PREDOCK"
        self.status = "RUNNING"
        self.reason = ""
        self.t_start = None
        self.t_phase = None
        self.blocked_since = None
        self.settle_ref = None
        self.measured_range = None
        self.range_source = ""
        self.final_lateral_error = None
        self.final_heading_error = None
        self.refined = line is not None
        self.refine_count = 0
        self.refined_center = None

    # ------------------------------------------------------------ helpers
    def _set_phase(self, phase, t):
        self.log(f"[{self.name}] {self.phase} -> {phase}")
        self.phase = phase
        self.t_phase = t

    def _fail(self, reason, t):
        self.status = "FAILED"
        self.reason = reason
        self.log(f"[{self.name}] FAILED: {reason}")
        return 0.0, 0.0

    @staticmethod
    def _sat(v, lim):
        return max(-lim, min(lim, v))

    def _range(self, pose, scan, angles):
        r = face_range(scan, angles)
        if r is not None:
            self.range_source = "lidar"
            return r
        self.range_source = "pose"
        return self.line.along_distance(pose[0], pose[1])

    # -------------------------------------------------------------- update
    def update(self, pose, scan, angles, t):
        """One control step -> (v, w). Check .status for RUNNING/DONE/FAILED."""
        if self.status != "RUNNING":
            return 0.0, 0.0
        if self.t_start is None:
            self.t_start = self.t_phase = t
        if t - self.t_start > config.ARM_DOCK_TIMEOUT:
            return self._fail(f"TIMEOUT in {self.phase}", t)
        if not all(math.isfinite(v) for v in pose):
            return self._fail("INVALID_POSE", t)
        return getattr(self, f"_do_{self.phase.lower()}")(pose, scan, angles, t)

    def _do_turn_to_target(self, pose, scan, angles, t):
        ex, ey = self.parcel_estimate
        target = math.atan2(ey - pose[1], ex - pose[0])
        err = wrap_to_pi(target - pose[2])
        if abs(err) < config.ARM_DOCK_HEADING_TOL * 2:
            self._set_phase("REFINE", t)
            return 0.0, 0.0
        return 0.0, self._sat(config.ARM_DOCK_K_HEADING * err, config.ARM_DOCK_W_MAX)

    def _do_refine(self, pose, scan, angles, t):
        line = refine_parcel_from_scan(pose, scan, angles, self.parcel_estimate,
                                       preferred_heading=self.preferred_heading)
        if line is None:
            return self._fail("PARCEL_FACE_NOT_FOUND_BY_LIDAR", t)
        self.refine_count += 1
        ux, uy = line.direction
        cx = line.face_point[0] + ux * config.PKG_SIZE / 2.0
        cy = line.face_point[1] + uy * config.PKG_SIZE / 2.0
        ex, ey = self.parcel_estimate
        self.log(f"[{self.name}] LiDAR fit #{self.refine_count}: parcel centre ({cx:.3f}, {cy:.3f}) "
                 f"vs estimate ({ex:.3f}, {ey:.3f}) -> shift "
                 f"{math.hypot(cx - ex, cy - ey) * 100:.1f} cm, "
                 f"approach heading {math.degrees(line.heading):.1f} deg")
        self.line = line
        self.refined_center = (cx, cy)
        self.refined = True
        self._set_phase("TO_PREDOCK", t)
        return 0.0, 0.0

    def _do_to_predock(self, pose, scan, angles, t):
        gx, gy, _ = self.line.pose_at(self.line.standoff + config.ARM_DOCK_PREDOCK_EXTRA)
        dx, dy = gx - pose[0], gy - pose[1]
        dist = math.hypot(dx, dy)
        if dist < config.ARM_DOCK_POINT_TOL:
            self._set_phase("FACE", t)
            return 0.0, 0.0
        bearing = math.atan2(dy, dx)
        err = wrap_to_pi(bearing - pose[2])
        reverse = abs(err) > math.pi / 2 and dist < 0.25
        if reverse:
            err = wrap_to_pi(err + math.pi)
        # do not drive into something while repositioning (forward only;
        # reversing is limited to <= 0.25 m inside the cleared dock area)
        if not reverse and scan is not None and angles is not None:
            if front_min_range(scan, angles, config.FRONT_CONE) < config.ARM_DOCK_MIN_RANGE:
                if self.blocked_since is None:
                    self.blocked_since = t
                if t - self.blocked_since > 5.0:
                    return self._fail("BLOCKED_WHILE_REPOSITIONING", t)
                return 0.0, 0.0
        self.blocked_since = None
        w = self._sat(config.ARM_DOCK_K_HEADING * err, config.ARM_DOCK_W_MAX)
        if abs(err) > 0.3:
            return 0.0, w
        v = min(config.ARM_DOCK_V_MAX, config.ARM_DOCK_K_RANGE * dist) * math.cos(err)
        return (-v if reverse else v), w

    def _do_face(self, pose, scan, angles, t):
        err = wrap_to_pi(self.line.heading - pose[2])
        if abs(err) < config.ARM_DOCK_HEADING_TOL:
            if self.parcel_estimate is not None and self.refine_count < 2:
                # coarse-to-fine: the first fit may come from an oblique view;
                # now the robot faces the parcel square-on from the pre-dock
                # point -> fit again and re-position if the line moved
                self.parcel_estimate = self.refined_center
                self._set_phase("REFINE", t)
                return 0.0, 0.0
            self._set_phase("CREEP", t)
            return 0.0, 0.0
        return 0.0, self._sat(config.ARM_DOCK_K_HEADING * err, config.ARM_DOCK_W_MAX)

    def _do_creep(self, pose, scan, angles, t):
        r = self._range(pose, scan, angles)
        self.measured_range = r
        if r < config.ARM_DOCK_MIN_RANGE:
            return self._fail(f"TOO_CLOSE ({r:.3f} m)", t)
        e_r = r - self.line.standoff
        e_lat = self.line.lateral_error(pose[0], pose[1])
        if abs(e_r) < config.ARM_DOCK_RANGE_TOL:
            self._set_phase("SETTLE", t)
            self.settle_ref = pose
            return 0.0, 0.0
        v = self._sat(config.ARM_DOCK_K_RANGE * e_r, config.ARM_DOCK_V_MAX)
        if abs(v) < 0.01:
            v = math.copysign(0.01, e_r)
        # steer back toward the line while moving (sign flips when reversing)
        desired = self.line.heading - math.copysign(1.0, v) * math.atan(
            config.ARM_DOCK_K_LATERAL * e_lat)
        w = self._sat(config.ARM_DOCK_K_HEADING * wrap_to_pi(desired - pose[2]),
                      config.ARM_DOCK_W_MAX)
        return v, w

    def _do_settle(self, pose, scan, angles, t):
        moved = math.hypot(pose[0] - self.settle_ref[0], pose[1] - self.settle_ref[1])
        if moved > config.ARM_DOCK_SETTLE_MOTION:
            self.settle_ref = pose
            self.t_phase = t
            return 0.0, 0.0
        if t - self.t_phase < config.ARM_DOCK_SETTLE_TIME:
            return 0.0, 0.0
        r = self._range(pose, scan, angles)
        self.measured_range = r
        e_lat = self.line.lateral_error(pose[0], pose[1])
        e_h = wrap_to_pi(self.line.heading - pose[2])
        self.final_lateral_error = e_lat
        self.final_heading_error = e_h
        if abs(e_lat) > config.ARM_DOCK_LATERAL_TOL:
            return self._fail(f"LATERAL_ERROR {e_lat * 100:.1f} cm", t)
        if abs(e_h) > 2 * config.ARM_DOCK_HEADING_TOL:
            return self._fail(f"HEADING_ERROR {math.degrees(e_h):.1f} deg", t)
        if abs(r - self.line.standoff) > 2 * config.ARM_DOCK_RANGE_TOL:
            # drifted while settling: creep again
            self._set_phase("CREEP", t)
            return 0.0, 0.0
        self.status = "DONE"
        self.log(f"[{self.name}] DOCKED: range {r:.3f} m ({self.range_source}), "
                 f"lateral {e_lat * 100:+.1f} cm, heading {math.degrees(e_h):+.1f} deg")
        return 0.0, 0.0
