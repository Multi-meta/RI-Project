"""LocalizationManager (EKF as the navigation pose): wiring, health, recovery.

Simulated trajectories with known ground truth; encoders, GPS and IMU are
corrupted with seeded noise. Checks the cases listed in the Phase-11 task:
straight line, turning, reverse driving, GPS noise, GPS dropout, IMU noise,
GPS outliers, gate rejection, covariance stability, the kidnapped-robot
recovery, NEES consistency, and the odometry-only vs GPS-only vs EKF RMSE
ordering.
"""

import math

import numpy as np
import pytest

import config
from kinematics import Odometry, wrap_to_pi
from localization import LocalizationManager

DT = 0.032


def simulate(mode="ekf", T=40.0, v_of_t=None, w_of_t=None, gps_sigma=0.05,
             imu_sigma=math.radians(1.0), odom_scale=0.03, gps_dropout=None,
             gps_outliers=(), displace=None, seed=1):
    rng = np.random.default_rng(seed)
    v_of_t = v_of_t or (lambda t: 0.3)
    w_of_t = w_of_t or (lambda t: 0.0)
    x = np.array([0.0, 0.0, 0.0])
    loc = LocalizationManager(mode, log=lambda m: None)
    errs, sig, nees, t = [], [], [], 0.0
    inc = (0.0, 0.0)
    jumps = 0
    while t < T:
        gps = None
        if not (gps_dropout and gps_dropout[0] <= t < gps_dropout[1]):
            gps = (x[0] + rng.normal(0, gps_sigma), x[1] + rng.normal(0, gps_sigma))
            for (to, dx, dy) in gps_outliers:
                if abs(t - to) < DT / 2:
                    gps = (gps[0] + dx, gps[1] + dy)
        yaw = wrap_to_pi(x[2] + rng.normal(0, imu_sigma))
        est = loc.step(t, inc, gps, yaw)
        if loc.consume_pose_jump() is not None:
            jumps += 1
        if est is not None and t > 1.0:
            e = np.array([est[0] - x[0], est[1] - x[1], wrap_to_pi(est[2] - x[2])])
            errs.append(e)
            P = loc.covariance()
            sig.append(math.sqrt(P[0, 0]))
            nees.append(float(e @ np.linalg.solve(P, e)))
        v, w = v_of_t(t), w_of_t(t)
        th = x[2] + w * DT
        x = np.array([x[0] + v * DT * math.cos(x[2] + w * DT / 2),
                      x[1] + v * DT * math.sin(x[2] + w * DT / 2), wrap_to_pi(th)])
        inc = (v * DT * (1 + odom_scale), w * DT * (1 + odom_scale))
        if displace and abs(t - displace[0]) < DT / 2:
            x = x + np.array(displace[1:])
        t += DT
    e = np.array(errs)
    return dict(rmse=float(np.sqrt(np.mean(e[:, 0] ** 2 + e[:, 1] ** 2))),
                final=float(np.hypot(*e[-1, :2])), sig=np.array(sig),
                nees=np.array(nees), loc=loc, jumps=jumps,
                rmse_th=float(np.sqrt(np.mean(e[:, 2] ** 2))))


def test_waits_for_first_valid_gps_and_imu():
    loc = LocalizationManager("ekf", log=lambda m: None)
    assert loc.step(0.0, (0, 0), (math.nan, math.nan), math.nan) is None
    assert loc.step(0.016, (0, 0), None, 0.1) is None
    pose = loc.step(0.032, (0, 0), (1.0, 2.0), 0.5)
    assert pose == pytest.approx((1.0, 2.0, 0.5))


def test_straight_line_ekf_beats_odometry_and_gps():
    res = {m: simulate(m) for m in ("odom", "gps", "ekf")}
    assert res["ekf"]["rmse"] < res["gps"]["rmse"]
    assert res["ekf"]["rmse"] < res["odom"]["rmse"]
    assert res["ekf"]["rmse"] < 0.05


def test_turning_trajectory():
    w = lambda t: 0.4 * math.sin(0.3 * t)            # noqa: E731
    res = {m: simulate(m, w_of_t=w) for m in ("odom", "gps", "ekf")}
    assert res["ekf"]["rmse"] < min(res["odom"]["rmse"], res["gps"]["rmse"])
    assert res["ekf"]["rmse_th"] < math.radians(2.0)


def test_reverse_driving_uses_signed_increments():
    """Old wiring predicted forward motion while reversing (hypot >= 0)."""
    r = simulate("odom", v_of_t=lambda t: -0.3, odom_scale=0.0, gps_sigma=0.0,
                 imu_sigma=0.0)
    assert r["final"] < 0.02


def test_odometry_reports_signed_increment():
    o = Odometry()
    o.update(0.0, 0.0)
    o.update(-1.0, -1.0)                              # both wheels backwards
    assert o.last_ds < 0.0 and o.pose()[0] < 0.0


def test_gps_dropout_dead_reckoning_and_recovery():
    r = simulate("ekf", gps_dropout=(10.0, 20.0))
    loc = r["loc"]
    # covariance grew during the dropout and shrank again afterwards
    i10, i19, end = int(9 / DT), int(18.9 / DT), -1
    assert r["sig"][i19] > 2 * r["sig"][i10]
    assert r["sig"][end] < r["sig"][i19]
    assert loc.status == "OK"
    assert r["rmse"] < 0.10


def test_status_reports_dead_reckoning_during_dropout():
    loc = LocalizationManager("ekf", log=lambda m: None)
    loc.step(0.0, (0, 0), (0.0, 0.0), 0.0)
    for k in range(1, 40):
        loc.step(k * DT, (0.01, 0.0), None, 0.0)
    assert loc.status == "DEAD_RECKONING"
    assert not loc.healthy() or loc.summary()["gps"] == "FAILED"


def test_imu_noise_is_filtered():
    r = simulate("ekf", imu_sigma=math.radians(3.0))
    assert r["rmse_th"] < math.radians(3.0)


def test_gps_outliers_are_rejected_and_do_not_pull_the_estimate():
    r = simulate("ekf", gps_outliers=[(15.0, 5.0, -4.0), (25.0, -3.0, 3.0)])
    assert r["loc"].n_gps_rejected >= 2
    assert r["jumps"] == 0                            # single glitches: no relocalisation
    assert r["rmse"] < 0.06


def test_covariance_stays_bounded_and_positive():
    r = simulate("ekf", T=120.0)
    P = r["loc"].covariance()
    assert np.all(np.linalg.eigvalsh(P) > 0)
    assert r["sig"][-1] < 0.1
    assert np.all(np.isfinite(r["sig"]))


def test_nees_is_consistent():
    """Average NEES of a consistent 3-state filter ~ 3 (chi^2, 3 DOF)."""
    r = simulate("ekf", T=60.0)
    assert 0.3 < float(np.mean(r["nees"])) < 9.0


def test_kidnapped_robot_relocalises_and_reports_the_jump():
    r = simulate("ekf", displace=(15.0, 0.8, -0.5, 0.0), gps_sigma=0.03)
    loc = r["loc"]
    assert loc.relocalizations == 1 and r["jumps"] == 1
    assert r["final"] < 0.08


def test_without_kidnap_recovery_the_filter_stays_lost(monkeypatch):
    """Documents the bug that was fixed: gate rejects every later GPS fix."""
    monkeypatch.setattr(config, "EKF_KIDNAP_TIME", math.inf)
    r = simulate("ekf", displace=(15.0, 0.8, -0.5, 0.0), gps_sigma=0.03, T=30.0)
    assert r["final"] > 0.5


def test_gps_trust_drops_and_r_inflates_with_rejections():
    loc = LocalizationManager("ekf", log=lambda m: None)
    loc.step(0.0, (0, 0), (0.0, 0.0), 0.0)
    rng = np.random.default_rng(3)
    for k in range(1, 60):
        bad = k % 2 == 0
        g = (rng.normal(0, 3.0), rng.normal(0, 3.0)) if bad else (0.0, 0.0)
        loc.step(k * DT, (0.0, 0.0), g, 0.0)
    assert loc.gps_health.status == "DEGRADED"
    assert loc.gps_health.r_inflation() > 1.0


def test_unknown_mode_rejected():
    with pytest.raises(ValueError):
        LocalizationManager("magic")
