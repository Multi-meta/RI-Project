"""
arm_kinematics.py — Kinematics and workspace analysis for Robot v2 mobile manipulator.

Per addendum §A9.3 & checklist 14.3:
- Planar 2R arm + lift (z) + wrist yaw:
  * Link 1 length: l1 = 0.30 m
  * Link 2 length: l2 = 0.28 m
  * Reach: up to 0.58 m
  * Lift range: 0.0 to 1.1 m
- Analytic IK:
  * Both elbow-up and elbow-down solutions
  * Joint limit checking (shoulder +-120 deg, elbow +-120 deg)
  * Out-of-reach rejection
- Forward kinematics (FK):
  * FK(lift_z, theta1, theta2, theta3) -> (x, y, z, yaw)
- Workspace coverage analysis for shelves and receiving pile.
"""

import math
from typing import Optional, Tuple, List


class ArmKinematics:
    """Analytical FK and IK for lift + 2R arm + wrist yaw."""

    def __init__(self, l1: float = 0.30, l2: float = 0.28,
                 lift_min: float = 0.0, lift_max: float = 1.1,
                 theta1_limits: Tuple[float, float] = (-2.094, 2.094),   # +-120 deg
                 theta2_limits: Tuple[float, float] = (-2.094, 2.094),   # +-120 deg
                 wrist_limits: Tuple[float, float] = (-3.14159, 3.14159)):
        self.l1 = l1
        self.l2 = l2
        self.lift_min = lift_min
        self.lift_max = lift_max
        self.theta1_limits = theta1_limits
        self.theta2_limits = theta2_limits
        self.wrist_limits = wrist_limits

    def forward_kinematics(self, lift_z: float, theta1: float, theta2: float, theta3: float) -> Tuple[float, float, float, float]:
        """
        Forward kinematics from joint angles to end-effector pose in arm-base frame.
        
        Returns:
            (x, y, z, yaw)
        """
        x = self.l1 * math.cos(theta1) + self.l2 * math.cos(theta1 + theta2)
        y = self.l1 * math.sin(theta1) + self.l2 * math.sin(theta1 + theta2)
        z = lift_z
        yaw = theta1 + theta2 + theta3
        return x, y, z, yaw

    def inverse_kinematics(self, target_x: float, target_y: float, target_z: float,
                           target_yaw: float = 0.0,
                           elbow_up: bool = False) -> Optional[Tuple[float, float, float, float]]:
        """
        Analytic inverse kinematics.
        
        Args:
            target_x, target_y: planar reach in arm-base frame (m)
            target_z: vertical height (m)
            target_yaw: desired end-effector orientation (rad)
            elbow_up: True for elbow-up (+theta2), False for elbow-down (-theta2)
            
        Returns:
            (lift_z, theta1, theta2, theta3) or None if unreachable
        """
        # 1. Lift check
        if not (self.lift_min <= target_z <= self.lift_max):
            return None
        lift_z = target_z

        # 2. Planar reach check
        r2 = target_x**2 + target_y**2
        c2 = (r2 - self.l1**2 - self.l2**2) / (2.0 * self.l1 * self.l2)

        # Out of reach or too close
        if abs(c2) > 1.0:
            return None

        # 3. Elbow angle (theta2)
        # c2 can slightly exceed 1.0 due to float rounding
        c2 = max(-1.0, min(1.0, c2))
        s2 = math.sqrt(max(0.0, 1.0 - c2**2))
        theta2 = math.atan2(s2 if elbow_up else -s2, c2)

        # 4. Shoulder angle (theta1)
        theta1 = math.atan2(target_y, target_x) - math.atan2(self.l2 * math.sin(theta2),
                                                            self.l1 + self.l2 * math.cos(theta2))

        # 5. Wrist yaw (theta3) to maintain desired orientation
        theta3 = target_yaw - (theta1 + theta2)

        # Wrap angles to [-pi, pi]
        theta1 = math.atan2(math.sin(theta1), math.cos(theta1))
        theta2 = math.atan2(math.sin(theta2), math.cos(theta2))
        theta3 = math.atan2(math.sin(theta3), math.cos(theta3))

        # 6. Joint limits verification
        if not (self.theta1_limits[0] <= theta1 <= self.theta1_limits[1]):
            return None
        if not (self.theta2_limits[0] <= theta2 <= self.theta2_limits[1]):
            return None
        if not (self.wrist_limits[0] <= theta3 <= self.wrist_limits[1]):
            return None

        return lift_z, theta1, theta2, theta3

    def get_both_solutions(self, target_x: float, target_y: float, target_z: float,
                           target_yaw: float = 0.0) -> List[Tuple[float, float, float, float]]:
        """Return all valid IK solutions (elbow-up and elbow-down)."""
        solutions = []
        for elbow_up in [False, True]:
            sol = self.inverse_kinematics(target_x, target_y, target_z, target_yaw, elbow_up=elbow_up)
            if sol is not None:
                solutions.append(sol)
        return solutions

    def is_reachable(self, x: float, y: float, z: float) -> bool:
        """Quick check if target is within reach."""
        if not (self.lift_min <= z <= self.lift_max):
            return False
        dist = math.hypot(x, y)
        min_reach = abs(self.l1 - self.l2)
        max_reach = self.l1 + self.l2
        return min_reach <= dist <= max_reach


# ===========================================================================
# Planar pitch arm of the mission robot (shoulder / elbow / wrist, all about
# the robot's y axis). This is the arm that is actually built into
# worlds/warehouse.wbt by tools/generate_world.py; ArmKinematics above is the
# earlier v2 (lift + SCARA) design study and is kept for its tests.
# ===========================================================================

def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class PlanarPitchArm:
    """Analytic FK / IK for a 3-joint planar arm moving in the robot x-z plane.

    Conventions (robot frame FLU, metres / radians):
        * the shoulder axis sits at (base_x, shoulder_z);
        * q1 (shoulder), q2 (elbow), q3 (wrist) are measured from +x,
          POSITIVE = UP (counter-clockwise when looking along -y);
        * the gripper pitch is phi = q1 + q2 + q3 (0 = horizontal, forward);
        * the tool centre point (TCP) is ``tool_len`` metres beyond the wrist
          along the gripper axis (here: the centre of a grasped parcel).

    FK:
        E   = S + L1 (cos q1,        sin q1)
        W   = E + L2 (cos(q1+q2),    sin(q1+q2))
        TCP = W + Lt (cos phi,       sin phi)
    IK (TCP x, z and phi given):
        W   = TCP - Lt (cos phi, sin phi);  d = |W - S|
        c2  = (d^2 - L1^2 - L2^2) / (2 L1 L2)        |c2| <= 1 else unreachable
        q2  = -acos(c2)  (elbow UP)   or  +acos(c2) (elbow DOWN)
        q1  = atan2(Wz - Sz, Wx - Sx) - atan2(L2 sin q2, L1 + L2 cos q2)
        q3  = phi - q1 - q2
    """

    def __init__(self, base_x: float, shoulder_z: float, l1: float, l2: float,
                 tool_len: float, limits: Tuple[Tuple[float, float], ...]):
        if l1 <= 0 or l2 <= 0 or tool_len < 0:
            raise ValueError("link lengths must be positive")
        if len(limits) != 3:
            raise ValueError("need (min, max) limits for 3 joints")
        self.sx = float(base_x)
        self.sz = float(shoulder_z)
        self.l1 = float(l1)
        self.l2 = float(l2)
        self.lt = float(tool_len)
        self.limits = tuple((float(lo), float(hi)) for lo, hi in limits)

    @classmethod
    def from_config(cls, cfg=None) -> "PlanarPitchArm":
        """Build the arm from config.py (single source of truth)."""
        if cfg is None:
            import config as cfg  # local import keeps this module importable alone
        return cls(cfg.ARM_BASE_X, cfg.ARM_SHOULDER_Z, cfg.ARM_UPPER_LEN,
                   cfg.ARM_FORE_LEN, cfg.ARM_TCP_OFFSET, cfg.ARM_JOINT_LIMITS_RAD)

    # ------------------------------------------------------------- geometry
    @property
    def shoulder(self) -> Tuple[float, float]:
        return self.sx, self.sz

    @property
    def max_reach(self) -> float:
        """Shoulder -> wrist distance at full stretch."""
        return self.l1 + self.l2

    @property
    def min_reach(self) -> float:
        return abs(self.l1 - self.l2)

    def forward(self, q) -> dict:
        """Joint angles -> key points {S, E, W, TCP} and gripper pitch ``phi``."""
        q1, q2, q3 = (float(v) for v in q)
        ex = self.sx + self.l1 * math.cos(q1)
        ez = self.sz + self.l1 * math.sin(q1)
        wx = ex + self.l2 * math.cos(q1 + q2)
        wz = ez + self.l2 * math.sin(q1 + q2)
        phi = q1 + q2 + q3
        tx = wx + self.lt * math.cos(phi)
        tz = wz + self.lt * math.sin(phi)
        return {"S": (self.sx, self.sz), "E": (ex, ez), "W": (wx, wz),
                "TCP": (tx, tz), "phi": phi}

    def within_limits(self, q, margin: float = 0.0) -> bool:
        return all(lo + margin <= v <= hi - margin
                   for v, (lo, hi) in zip(q, self.limits))

    def clamp(self, q) -> Tuple[float, float, float]:
        """Clamp a joint vector into the configured limits (safety layer)."""
        return tuple(min(max(float(v), lo), hi)
                     for v, (lo, hi) in zip(q, self.limits))

    def inverse(self, x: float, z: float, phi: float = 0.0,
                elbow_up: bool = True, check_limits: bool = True):
        """TCP (x, z, phi) -> (q1, q2, q3) or None.

        Returns None when the wrist point is outside the annulus
        [|L1 - L2|, L1 + L2] or (with ``check_limits``) when the solution
        violates a joint limit. Use :meth:`solve` for a diagnostic reason.
        """
        q, _reason = self.solve(x, z, phi, elbow_up, check_limits)
        return q

    def solve(self, x: float, z: float, phi: float = 0.0,
              elbow_up: bool = True, check_limits: bool = True):
        """Like :meth:`inverse` but returns ``(q | None, reason)``.

        reason is "" on success, otherwise "OUT_OF_REACH" or
        "JOINT_LIMIT(<joint>)".
        """
        if not all(math.isfinite(v) for v in (x, z, phi)):
            return None, "INVALID_TARGET"
        wx = x - self.lt * math.cos(phi)
        wz = z - self.lt * math.sin(phi)
        dx, dz = wx - self.sx, wz - self.sz
        d2 = dx * dx + dz * dz
        c2 = (d2 - self.l1 ** 2 - self.l2 ** 2) / (2.0 * self.l1 * self.l2)
        if c2 > 1.0 + 1e-12 or c2 < -1.0 - 1e-12:
            return None, "OUT_OF_REACH"
        c2 = max(-1.0, min(1.0, c2))
        q2 = math.acos(c2)
        if elbow_up:
            q2 = -q2
        q1 = math.atan2(dz, dx) - math.atan2(self.l2 * math.sin(q2),
                                             self.l1 + self.l2 * math.cos(q2))
        q1 = _wrap(q1)
        q3 = _wrap(phi - q1 - q2)
        q = (q1, q2, q3)
        if check_limits:
            for name, v, (lo, hi) in zip(("shoulder", "elbow", "wrist"), q,
                                         self.limits):
                if not (lo <= v <= hi):
                    return None, f"JOINT_LIMIT({name})"
        return q, ""

    def reach_ratio(self, x: float, z: float, phi: float = 0.0) -> float:
        """|shoulder -> wrist| / (L1 + L2) for a TCP target (1.0 = singular)."""
        wx = x - self.lt * math.cos(phi)
        wz = z - self.lt * math.sin(phi)
        return math.hypot(wx - self.sx, wz - self.sz) / self.max_reach

    def link_segments(self, q, wrist_to_palm: float, finger_len: float):
        """Arm as named 2-D segments in the x-z plane (clearance checks, plots).

        Returns [(p0, p1, name)] for upper_arm, forearm, palm (wrist -> palm
        face) and fingers (palm face -> finger tips), all along the gripper axis.
        """
        p = self.forward(q)
        c, s = math.cos(p["phi"]), math.sin(p["phi"])
        wx, wz = p["W"]
        palm = (wx + wrist_to_palm * c, wz + wrist_to_palm * s)
        tips = (palm[0] + finger_len * c, palm[1] + finger_len * s)
        return [(p["S"], p["E"], "upper_arm"),
                (p["E"], p["W"], "forearm"),
                (p["W"], palm, "palm"),
                (palm, tips, "fingers")]


def segment_rect_distance(p0, p1, rect) -> float:
    """Minimum distance between segment p0-p1 and an axis-aligned rectangle.

    rect = (x_min, x_max, z_min, z_max). Returns 0 when they intersect.
    Pure 2-D geometry used by the arm clearance checks (x-z plane).
    """
    x0, x1, z0, z1 = rect

    def inside(p):
        return x0 <= p[0] <= x1 and z0 <= p[1] <= z1

    if inside(p0) or inside(p1):
        return 0.0
    corners = [(x0, z0), (x1, z0), (x1, z1), (x0, z1)]
    edges = list(zip(corners, corners[1:] + corners[:1]))
    for a, b in edges:
        if _segments_intersect(p0, p1, a, b):
            return 0.0
    dists = [_point_segment_distance(c, p0, p1) for c in corners]
    dists += [_point_segment_distance(p, a, b) for p in (p0, p1) for a, b in edges]
    return min(dists)


def _point_segment_distance(p, a, b) -> float:
    ax, az = a
    bx, bz = b
    px, pz = p
    vx, vz = bx - ax, bz - az
    L2 = vx * vx + vz * vz
    if L2 < 1e-18:
        return math.hypot(px - ax, pz - az)
    t = max(0.0, min(1.0, ((px - ax) * vx + (pz - az) * vz) / L2))
    return math.hypot(px - (ax + t * vx), pz - (az + t * vz))


def _segments_intersect(p1, p2, p3, p4) -> bool:
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    d1 = orient(p3, p4, p1)
    d2 = orient(p3, p4, p2)
    d3 = orient(p1, p2, p3)
    d4 = orient(p1, p2, p4)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


# ---------------------------------------------------------------------------
# Clearance model (x-z plane through the arm, y = 0)
# ---------------------------------------------------------------------------
# Half thickness of each arm part (m) as seen in the x-z plane.
ARM_PART_HALF_THICKNESS = {"upper_arm": 0.015, "forearm": 0.015,
                           "palm": 0.025, "fingers": 0.02}


def robot_obstacle_rects(cfg=None) -> dict:
    """Robot parts the arm / carried parcel must never touch, as x-z rects.

    All numbers come from config.py (body, bumpers, camera housings, post).
    Rect = (x_min, x_max, z_min, z_max) in the robot frame.
    """
    if cfg is None:
        import config as cfg
    hl = cfg.BODY_LENGTH / 2.0
    body_z0 = cfg.CASTER_RADIUS
    body_z1 = cfg.CASTER_RADIUS + cfg.BODY_HEIGHT
    bz = body_z0 + cfg.BODY_HEIGHT / 2.0
    bh = cfg.BODY_HEIGHT * 0.6                       # bumper height (generator)
    hd, _hw, hh = cfg.CAMERA_HOUSING_SIZE
    cz = cfg.CAMERA_MOUNT_Z
    ps = cfg.ARM_POST_SIZE / 2.0
    return {
        "body": (-hl, hl, body_z0, body_z1),
        "front_bumper": (hl, hl + 0.02, bz - bh / 2.0, bz + bh / 2.0),
        "rear_bumper": (-hl - 0.02, -hl, bz - bh / 2.0, bz + bh / 2.0),
        "front_camera": (cfg.CAMERA_MOUNT_X - hd, cfg.CAMERA_MOUNT_X,
                         cz - hh / 2.0, cz + hh / 2.0),
        "rear_camera": (cfg.REAR_CAMERA_MOUNT_X, cfg.REAR_CAMERA_MOUNT_X + hd,
                        cz - hh / 2.0, cz + hh / 2.0),
        "arm_post": (cfg.ARM_BASE_X - ps, cfg.ARM_BASE_X + ps, body_z1,
                     cfg.ARM_SHOULDER_Z - 0.02),
        "floor": (-2.0, 2.0, -0.5, 0.0),
    }


def parcel_rect(tcp, size) -> Tuple[float, float, float, float]:
    """x-z rect of a parcel whose centre is at the TCP (gripper horizontal)."""
    h = size / 2.0
    return (tcp[0] - h, tcp[0] + h, tcp[1] - h, tcp[1] + h)


def _rects_overlap(a, b, margin=0.0) -> bool:
    return not (a[1] + margin <= b[0] or b[1] + margin <= a[0]
                or a[3] + margin <= b[2] or b[3] + margin <= a[2])


def arm_clearance(arm: "PlanarPitchArm", q, *, wrist_to_palm: float,
                  finger_len: float, holding_size: Optional[float] = None,
                  obstacles: Optional[dict] = None,
                  target_parcel: Optional[Tuple[float, float, float, float]] = None):
    """Minimum clearance (m) of the arm (+ carried parcel) to obstacles.

    obstacles     : name -> rect; defaults to :func:`robot_obstacle_rects`.
    holding_size  : edge length of a parcel held at the TCP (None = empty).
    target_parcel : rect of a parcel being approached (not yet held): the
                    links and palm must not hit it; the fingers are allowed to
                    straddle it (they are at y = +-open half width, outside the
                    parcel), so they are excluded.

    Returns (clearance, culprit) with clearance < 0 meaning interpenetration.
    Exclusions that are part of the design: the upper arm starts on the post
    (shoulder bearing) and the floor is only checked for the parcel / fingers.
    """
    if obstacles is None:
        obstacles = robot_obstacle_rects()
    segs = arm.link_segments(q, wrist_to_palm, finger_len)
    tcp = arm.forward(q)["TCP"]
    worst, culprit = math.inf, ""
    for p0, p1, name in segs:
        r = ARM_PART_HALF_THICKNESS[name]
        if name == "palm":
            # the palm is a flat plate: its half thickness applies across the
            # gripper axis, not beyond the palm face -> end the capsule there
            L = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            if L > r:
                k = (L - r) / L
                p1 = (p0[0] + (p1[0] - p0[0]) * k, p0[1] + (p1[1] - p0[1]) * k)
        for oname, rect in obstacles.items():
            if name == "upper_arm" and oname == "arm_post":
                continue
            c = segment_rect_distance(p0, p1, rect) - r
            if c < worst:
                worst, culprit = c, f"{name}/{oname}"
        if target_parcel is not None and name != "fingers":
            c = segment_rect_distance(p0, p1, target_parcel) - r
            if c < worst:
                worst, culprit = c, f"{name}/target_parcel"
    if holding_size is not None:
        prect = parcel_rect(tcp, holding_size)
        for p0, p1, name in segs:
            if name in ("upper_arm", "forearm"):
                c = segment_rect_distance(p0, p1, prect) - ARM_PART_HALF_THICKNESS[name]
                if c < worst:
                    worst, culprit = c, f"{name}/held_parcel"
        for oname, rect in obstacles.items():
            c = _rect_gap(prect, rect)
            if c < worst:
                worst, culprit = c, f"held_parcel/{oname}"
    return worst, culprit


def _rect_gap(a, b) -> float:
    """Signed gap between two rects (negative = overlap depth, approx.)."""
    dx = max(b[0] - a[1], a[0] - b[1])
    dz = max(b[2] - a[3], a[2] - b[3])
    if dx < 0 and dz < 0:
        return max(dx, dz)
    return math.hypot(max(dx, 0.0), max(dz, 0.0))
