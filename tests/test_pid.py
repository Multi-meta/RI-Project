"""PID tests: proportional response, integral clamp, derivative sign, reset."""

import math

import pytest

from pid import PID


def test_proportional():
    p = PID(2.0, 0.0, 0.0)
    assert p.update(1.0, 0.1) == pytest.approx(2.0)


def test_integral_accumulates_and_clamps():
    p = PID(0.0, 1.0, 0.0, out_min=-1.0, out_max=1.0)
    u = p.update(1.0, 0.1)          # integral = 0.1
    assert u == pytest.approx(0.1)
    for _ in range(100):            # keep integrating -> saturates
        u = p.update(1.0, 0.1)
    assert u == pytest.approx(1.0)  # clamped to out_max


def test_anti_windup_recovers():
    p = PID(0.0, 1.0, 0.0, out_min=-1.0, out_max=1.0)
    for _ in range(200):            # saturate
        p.update(1.0, 0.1)
    # error flips sign; output must come OFF the limit quickly (not stay
    # pinned while a wound-up integral unwinds for many steps)
    left_limit = False
    for _ in range(20):
        u = p.update(-1.0, 0.1)
        if abs(u) < 1.0:
            left_limit = True
            break
    assert left_limit


def test_derivative_sign():
    p = PID(0.0, 0.0, 1.0)
    p.update(1.0, 0.1)              # prime prev_error (deriv = 0 first step)
    assert p.update(2.0, 0.1) == pytest.approx(10.0)   # error grew -> positive
    assert p.update(1.0, 0.1) == pytest.approx(-10.0)  # error shrank -> negative


def test_reset():
    p = PID(1.0, 1.0, 1.0, out_min=-1, out_max=1)
    for _ in range(50):
        p.update(1.0, 0.1)
    p.reset()
    assert p.integral == 0.0
    assert p.update(0.0, 0.1) == pytest.approx(0.0)


def test_invalid_dt():
    p = PID(1.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        p.update(1.0, 0.0)
