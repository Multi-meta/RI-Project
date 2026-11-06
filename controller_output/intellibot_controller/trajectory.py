"""
trajectory.py — Quintic polynomial trajectory generation for manipulator joints.

Per addendum §A9.3 & checklist 14.4:
- Quintic profile: s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5, tau in [0, 1]
- Smooth motion: zero start and end velocity and acceleration.
- Multi-joint coordination: calculates minimum execution time based on joint speed/acc limits.
- Cartesian waypoint interpolation via IK.
"""

import math
from typing import List, Tuple, Optional
import numpy as np


def quintic_profile(tau: float) -> Tuple[float, float, float]:
    """
    Evaluate normalized quintic profile at tau in [0, 1].
    
    Returns:
        (s, s_dot, s_ddot) normalized position, velocity, acceleration factors.
    """
    tau = max(0.0, min(1.0, tau))
    tau2 = tau * tau
    tau3 = tau2 * tau
    tau4 = tau3 * tau
    tau5 = tau4 * tau

    s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5
    s_dot = 30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4
    s_ddot = 60.0 * tau - 180.0 * tau2 + 120.0 * tau3
    return s, s_dot, s_ddot


class QuinticTrajectory:
    """Multi-joint trajectory generator using quintic polynomial segments."""

    def __init__(self, q_start: np.ndarray, q_goal: np.ndarray,
                 duration: Optional[float] = None,
                 max_velocity: float = 1.0,
                 max_acceleration: float = 2.0):
        self.q0 = np.asarray(q_start, dtype=float)
        self.q1 = np.asarray(q_goal, dtype=float)
        self.max_v = max_velocity
        self.max_a = max_acceleration

        if duration is not None and duration > 0:
            self.T = float(duration)
        else:
            self.T = self._compute_min_duration()

    def _compute_min_duration(self) -> float:
        """Compute duration needed to respect velocity and acceleration limits."""
        delta = np.abs(self.q1 - self.q0)
        # Max velocity factor of quintic profile is 15/8 = 1.875 at tau=0.5
        # v_max = 1.875 * |delta| / T  ==>  T_v = 1.875 * |delta| / max_v
        # Max accel factor of quintic is 10 / sqrt(3) ~= 5.77 at tau=(1 +- 1/sqrt(3))/2
        # a_max = 5.77 * |delta| / T^2 ==> T_a = sqrt(5.77 * |delta| / max_a)
        t_v = 1.875 * np.max(delta) / self.max_v
        t_a = math.sqrt(5.77 * np.max(delta) / self.max_a) if self.max_a > 0 else 0.0
        return max(0.1, float(max(t_v, t_a)))

    def sample(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Sample trajectory at time t.
        
        Returns:
            (position, velocity, acceleration) vectors
        """
        if self.T <= 0:
            return self.q1.copy(), np.zeros_like(self.q1), np.zeros_like(self.q1)

        tau = t / self.T
        s, s_dot, s_ddot = quintic_profile(tau)

        pos = self.q0 + (self.q1 - self.q0) * s
        vel = (self.q1 - self.q0) * (s_dot / self.T)
        acc = (self.q1 - self.q0) * (s_ddot / (self.T**2))
        return pos, vel, acc

    def is_finished(self, t: float) -> bool:
        return t >= self.T
