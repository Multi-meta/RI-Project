"""kinematics.py — differential-drive kinematics, odometry, angle helpers.

Pure Python (no Webots imports) so it is unit-testable and explainable.
Conventions: FLU robot frame (x forward, y left, z up), ENU world
(x east, y north, z up), yaw = rotation about +z, CCW positive.

Formulas (r = wheel radius, L = wheel separation):
    Forward:  v = r/2 (wR + wL)        w = r/L (wR - wL)
    Inverse:  wR = (v + w L/2) / r     wL = (v - w L/2) / r
Odometry (arc model, per step, dphi from encoder deltas):
    dsR = r dphiR, dsL = r dphiL, ds = (dsR+dsL)/2, dtheta = (dsR-dsL)/L
    x  += ds cos(theta + dtheta/2)
    y  += ds sin(theta + dtheta/2)
    theta += dtheta
"""

import math

import controller_output.intellibot_controller.config as config


def wrap_to_pi(a):
    """Wrap angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def forward_kinematics(w_left, w_right):
    """Wheel angular speeds (rad/s) -> body (v [m/s], w [rad/s])."""
    r, L = _params()
    v = r / 2.0 * (w_right + w_left)
    w = r / L * (w_right - w_left)
    return v, w


def inverse_kinematics(v, w):
    """Body twist -> wheel angular speeds, clamped to the motor max.

    Clamping scales BOTH wheels by the same factor so the commanded
    curvature (v/w ratio) is preserved. Returns (w_left, w_right) rad/s.
    """
    r, L = _params()
    w_left = (v - w * L / 2.0) / r
    w_right = (v + w * L / 2.0) / r
    max_w = _max_wheel_speed()
    scale = max(abs(w_left), abs(w_right))
    if scale > max_w:
        f = max_w / scale
        w_left *= f
        w_right *= f
    return w_left, w_right


class Odometry:
    """Wheel-odometry integrator using cumulative encoder angles (rad).

    Feed cumulative wheel angles each step (PositionSensor.getValue()).
    NaN-safe: readings before the first valid sample are ignored, so the
    Webots first-step NaN never poisons the state.
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta
        self._prev_left = None
        self._prev_right = None

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.__init__(x, y, theta)

    def update(self, phi_left, phi_right):
        """Integrate one step. Returns True if the step was applied."""
        if phi_left is None or phi_right is None:
            return False
        if math.isnan(phi_left) or math.isnan(phi_right):
            return False                      # first Webots step may be NaN
        if self._prev_left is None:           # establish reference, no motion
            self._prev_left, self._prev_right = phi_left, phi_right
            return False
        r, L = _params()
        dsl = r * (phi_left - self._prev_left)
        dsr = r * (phi_right - self._prev_right)
        self._prev_left, self._prev_right = phi_left, phi_right
        ds = (dsl + dsr) / 2.0
        dtheta = (dsr - dsl) / L
        self.x += ds * math.cos(self.theta + dtheta / 2.0)
        self.y += ds * math.sin(self.theta + dtheta / 2.0)
        self.theta = wrap_to_pi(self.theta + dtheta)
        return True

    def pose(self):
        return self.x, self.y, self.theta


# --- config access isolated here so tests can monkeypatch dimensions -------
_R, _L_, _WMAX = config.WHEEL_RADIUS, config.WHEEL_SEPARATION, config.MAX_WHEEL_SPEED


def set_params(r, separation, max_wheel_speed):
    """Override module parameters (tests only)."""
    global _R, _L_, _WMAX
    _R, _L_, _WMAX = r, separation, max_wheel_speed


def _params():
    return _R, _L_


def _max_wheel_speed():
    return _WMAX
