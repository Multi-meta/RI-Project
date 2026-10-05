"""pid.py — PID controller with output limits and integral anti-windup.

Pure Python. Used for heading control of the waypoint follower.

    u = kp*e + ki*integral + kd*derivative

Anti-windup: CONDITIONAL INTEGRATION — while the output is saturated, the
integrator is frozen if the error would push it further into saturation.
This keeps the response from winding up and recovers cleanly when the
error changes sign. The derivative acts on the error change per step.
"""

import math


class PID:
    def __init__(self, kp, ki, kd, out_min=-math.inf, out_max=math.inf):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_error = 0.0
        self._has_prev = False

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self._has_prev = False

    def update(self, error, dt):
        """One control step. dt > 0 in seconds. Returns clamped output."""
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        deriv = 0.0
        if self._has_prev:
            deriv = (error - self.prev_error) / dt
        self.prev_error = error
        self._has_prev = True

        i_trial = self.integral + error * dt
        u = self.kp * error + self.ki * i_trial + self.kd * deriv
        # anti-windup: freeze the integrator if we're saturated AND the error
        # would push the output further beyond the limit
        if (u > self.out_max and error > 0.0) or \
           (u < self.out_min and error < 0.0):
            i_trial = self.integral
            u = self.kp * error + self.ki * i_trial + self.kd * deriv
        self.integral = i_trial
        return min(max(u, self.out_min), self.out_max)
