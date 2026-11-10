"""safety.py — stopping distance, swept-corridor clearance and the speed governor.

Pure Python (no Webots). Used by DecisionMaker on EVERY base command, so the
mission logic can never command a speed the robot cannot stop from.

Stopping distance (measured from the robot's front/rear extent):

    d_stop(v) = v^2 / (2 a_brake) + v t_react + d_margin

with a_brake = SAFETY_BRAKE_DECEL (a conservative deceleration for the
wheel motors and the base mass with the arm), t_react = one perception + one
control period (SAFETY_REACTION_TIME) and d_margin = SAFETY_STOP_MARGIN.
Inverting it gives the clearance-limited speed

    v_clear(c) = a ( -t + sqrt(t^2 + 2 (c - d_margin) / a) )    (0 if c <= d_margin)

Speed cap (the user specification):

    v_cap = min(V_CRUISE, v_clear, v_health, v_zone)

* v_clear  : swept-corridor clearance in the DIRECTION OF TRAVEL (front for
             v > 0, rear for v < 0): rays whose lateral offset is inside the
             robot half width + SAFETY_LATERAL_MARGIN, minus the body extent;
* v_health : LiDAR invalid-ray fraction / localisation status caps
             (degraded sensors -> crawl, failed -> stop);
* v_zone   : carrying a parcel, final approach to a goal (docking is done by
             docking.Docker at its own low speeds).

Command shaping: the (v, w) pair is scaled by ONE factor so the curvature
w / v of the follower's arc is preserved, then v is rate-limited with
SAFETY_ACCEL (speeding up) / SAFETY_BRAKE_DECEL (slowing down); an emergency
(clearance below the margin) bypasses the comfort ramp and stops at once.
"""

import math

import numpy as np

import config


def stopping_distance(v, a_brake=None, t_react=None, margin=None):
    a = config.SAFETY_BRAKE_DECEL if a_brake is None else a_brake
    t = config.SAFETY_REACTION_TIME if t_react is None else t_react
    m = config.SAFETY_STOP_MARGIN if margin is None else margin
    v = abs(v)
    return v * v / (2.0 * a) + v * t + m


def clearance_speed(clearance, a_brake=None, t_react=None, margin=None):
    """Largest speed whose stopping distance fits in `clearance` (m)."""
    a = config.SAFETY_BRAKE_DECEL if a_brake is None else a_brake
    t = config.SAFETY_REACTION_TIME if t_react is None else t_react
    m = config.SAFETY_STOP_MARGIN if margin is None else margin
    if not math.isfinite(clearance):
        return math.inf
    c = clearance - m
    if c <= 0.0:
        return 0.0
    return a * (-t + math.sqrt(t * t + 2.0 * c / a))


def corridor_clearance(scan, angles, reverse=False, half_width=None, extent=None,
                       max_range=None):
    """Free distance ahead of the body (or behind when reverse) inside the
    swept corridor. Returns inf when nothing is in the corridor."""
    if scan is None or angles is None:
        return math.inf
    r = np.asarray(scan, dtype=float)
    a = np.asarray(angles, dtype=float)
    hw = (config.BODY_WIDTH / 2.0 + config.SAFETY_LATERAL_MARGIN
          if half_width is None else half_width)
    ext = config.ROBOT_FRONT_EXTENT if extent is None else extent
    rmax = config.LIDAR_MAX_RANGE if max_range is None else max_range
    ok = np.isfinite(r) & (r > 0.0) & (r < rmax)
    if not np.any(ok):
        return math.inf
    r, a = r[ok], a[ok]
    lon = r * np.cos(a)
    lat = r * np.sin(a)
    if reverse:
        lon = -lon
    inside = (lon > 0.0) & (np.abs(lat) <= hw)
    if not np.any(inside):
        return math.inf
    return float(np.min(lon[inside]) - ext)


def lidar_invalid_fraction(scan):
    if scan is None:
        return 1.0
    r = np.asarray(scan, dtype=float)
    if r.size == 0:
        return 1.0
    return float(np.mean(np.isnan(r)))


class SpeedGovernor:
    """Stateful (v, w) limiter: one instance per robot, call limit() each step."""

    def __init__(self, enabled=None, v_cruise=None):
        self.enabled = config.SPEED_GOVERNOR_ENABLED if enabled is None else enabled
        self.v_cruise = config.V_CRUISE if v_cruise is None else v_cruise
        self.v_prev = 0.0
        self.w_prev = 0.0
        self.last = {}
        self.min_clearance = math.inf
        self.emergency_stops = 0
        self.near_misses = 0
        self._in_emergency = False

    def reset(self):
        self.v_prev = self.w_prev = 0.0

    def limit(self, v, w, dt, scan=None, angles=None, mode="NAV",
              health=None, carrying=False, goal_dist=None, urgent=False):
        """Shape one command. mode: NAV (full checks) | DOCK (docking/arm:
        speed already tiny; only the ramp and health caps apply).
        urgent: stop NOW (bumper contact, follower safety stop) — no ramp."""
        health = health or {}
        if not self.enabled:
            self.v_prev, self.w_prev = v, w
            self.last = dict(v_cap=math.inf, reason="governor disabled", clearance=math.nan)
            return v, w
        reverse = v < 0.0
        clearance = corridor_clearance(scan, angles, reverse=reverse)
        if mode == "NAV" and math.isfinite(clearance):
            self.min_clearance = min(self.min_clearance, clearance)
        caps = {"cruise": self.v_cruise}
        if mode == "NAV":
            caps["clearance"] = clearance_speed(clearance)
            if carrying:
                caps["carrying"] = config.V_CARRY
            if goal_dist is not None and goal_dist < config.SAFETY_APPROACH_DIST:
                caps["approach"] = max(config.V_APPROACH_MIN,
                                       self.v_cruise * goal_dist / config.SAFETY_APPROACH_DIST)
        frac = lidar_invalid_fraction(scan) if scan is not None else 0.0
        if health.get("lidar") == "FAILED" or frac >= config.SAFETY_LIDAR_FAILED_FRAC:
            caps["lidar_failed"] = 0.0
        elif health.get("lidar") == "DEGRADED" or frac >= config.SAFETY_LIDAR_DEGRADED_FRAC:
            caps["lidar_degraded"] = config.V_DEGRADED
        loc = health.get("localization")
        if loc in ("DEAD_RECKONING", "GPS_REJECTED", "UNINITIALIZED"):
            caps["localization"] = config.V_DEGRADED
        reason = min(caps, key=caps.get)
        v_cap = caps[reason]

        emergency = urgent or (mode == "NAV" and abs(v) > 1e-6
                               and clearance <= config.SAFETY_STOP_MARGIN)
        # curvature-preserving clamp
        scale = 1.0
        if abs(v) > v_cap:
            scale = v_cap / abs(v) if abs(v) > 1e-9 else 0.0
        v_t, w_t = v * scale, w * scale if abs(v) > 1e-6 else w
        if v_cap <= 0.0 and reason == "lidar_failed":
            w_t = 0.0                           # blind: do not even turn
        # acceleration limits (emergency bypasses the comfort ramp)
        if emergency:
            v_out = 0.0
            w_out = 0.0
            if not self._in_emergency and not urgent:
                self.emergency_stops += 1
            self._in_emergency = True
        else:
            self._in_emergency = False
            dv = v_t - self.v_prev
            speeding_up = abs(v_t) > abs(self.v_prev) and v_t * self.v_prev >= 0.0
            lim = (config.SAFETY_ACCEL if speeding_up else config.SAFETY_BRAKE_DECEL) * dt
            v_out = self.v_prev + max(-lim, min(lim, dv))
            # keep the arc: if the ramp shortened v, shorten w by the same factor
            if abs(v_t) > 1e-6 and abs(v_out) < abs(v_t):
                w_out = w_t * (v_out / v_t)
            else:
                w_out = w_t
            wl = config.SAFETY_W_ACCEL * dt
            w_out = self.w_prev + max(-wl, min(wl, w_out - self.w_prev))
        # never ask for more than the wheels can do: scale (v, w) together so
        # |v| + |w| L/2 stays inside SAFETY_WHEEL_HEADROOM of the motor limit
        # (inverse_kinematics would clamp too, but then the ramp state above
        # would no longer match what the robot actually does)
        v_wheel_max = config.SAFETY_WHEEL_HEADROOM * config.MAX_WHEEL_SPEED * config.WHEEL_RADIUS
        need = abs(v_out) + abs(w_out) * config.WHEEL_SEPARATION / 2.0
        if need > v_wheel_max > 0.0:
            k = v_wheel_max / need
            v_out, w_out = v_out * k, w_out * k
        if mode == "NAV" and math.isfinite(clearance) and clearance < config.SAFETY_NEAR_MISS:
            self.near_misses += 1
        self.v_prev, self.w_prev = v_out, w_out
        self.last = dict(v_cap=v_cap, reason=reason, clearance=clearance,
                         emergency=emergency, d_stop=stopping_distance(v_out))
        return v_out, w_out

    def summary(self):
        return dict(min_clearance_m=(round(self.min_clearance, 3)
                                     if math.isfinite(self.min_clearance) else None),
                    emergency_stops=self.emergency_stops,
                    near_miss_steps=self.near_misses,
                    governor_enabled=self.enabled, v_cruise=self.v_cruise)
