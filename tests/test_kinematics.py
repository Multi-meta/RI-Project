"""Kinematics tests: straight line, spin, round-trip, clamping, odometry."""

import math

import pytest

import kinematics as k


def test_straight_line():
    v, w = k.forward_kinematics(2.0, 2.0)   # equal wheel speeds
    assert w == pytest.approx(0.0)
    assert v == pytest.approx(2.0 * 0.05)


def test_spin_in_place():
    v, w = k.forward_kinematics(-2.0, 2.0)  # opposite wheels
    assert v == pytest.approx(0.0)
    assert w == pytest.approx(0.05 / 0.24 * 4.0)


def test_round_trip():
    wl, wr = k.inverse_kinematics(0.3, 0.5)
    v, w = k.forward_kinematics(wl, wr)
    assert v == pytest.approx(0.3)
    assert w == pytest.approx(0.5)


def test_clamping_preserves_curvature():
    # command far beyond what the motors can deliver
    v, w = 5.0, 2.0                          # curvature = w/v = 0.4
    wl, wr = k.inverse_kinematics(v, w)
    assert max(abs(wl), abs(wr)) <= 12.0 + 1e-9
    v2, w2 = k.forward_kinematics(wl, wr)
    assert w2 / v2 == pytest.approx(w / v, rel=1e-6)   # same curvature
    assert v2 < v                                     # and slower


def test_wrap_to_pi():
    assert k.wrap_to_pi(math.pi) == pytest.approx(math.pi, abs=1e-9)
    assert abs(k.wrap_to_pi(3 * math.pi)) == pytest.approx(math.pi)
    assert abs(k.wrap_to_pi(-3 * math.pi)) == pytest.approx(math.pi)
    assert k.wrap_to_pi(0.1) == pytest.approx(0.1)


def test_odometry_line():
    o = k.Odometry()
    # drive straight: both wheels advance the same angle each step.
    # The first valid reading only establishes the reference, so motion is
    # integrated over steps-1 deltas.
    step_phi = 0.01
    pl = pr = 0.0
    for _ in range(100):
        pl += step_phi
        pr += step_phi
        o.update(pl, pr)
    x, y, th = o.pose()
    assert th == pytest.approx(0.0, abs=1e-9)
    assert x == pytest.approx(99 * step_phi * 0.05, rel=1e-6)
    assert y == pytest.approx(0.0, abs=1e-9)


def test_odometry_nan_first_step():
    o = k.Odometry()
    assert o.update(float("nan"), 0.0) is False   # ignored, no poison
    pl = pr = 0.0
    for i in range(1, 11):
        pl += 0.1
        pr += 0.1
        o.update(pl, pr)
    x, _, _ = o.pose()
    assert x == pytest.approx(9 * 0.1 * 0.05, rel=1e-6)  # 10th reading is ref


def test_odometry_circle():
    """Integrate a known circle: r_circle = v/w."""
    o = k.Odometry()
    v, w = 0.2, 0.5
    r_circ = v / w
    dt = 0.01
    steps = int(2 * math.pi / w / dt)             # one full revolution
    wl, wr = k.inverse_kinematics(v, w)
    # integrate wheel ANGLES with the same step
    dphi_l = wl * dt
    dphi_r = wr * dt
    pl = pr = 0.0
    first = True
    for _ in range(steps):
        nl, nr = pl + dphi_l, pr + dphi_r
        if first:
            o.update(nl, nr)
            first = False
        else:
            o.update(nl, nr)
        pl, pr = nl, nr
    x, y, th = o.pose()
    # arc-model integration of a circle is approximate: small numerical drift
    assert math.hypot(x, y) < 0.02
    assert abs(th) < 0.02
