"""Checklist 11.5: camera bearing update with R scaled by light level."""

import math

import numpy as np
import pytest

import config
from camera_pipeline import camera_bearing_variance
from ekf_localization import EKFLocalizer


def ekf():
    return EKFLocalizer(np.array([0.0, 0.0, 0.0]), np.diag([0.05, 0.05, 0.05]),
                        np.diag(config.EKF_Q))


def test_variance_grows_as_light_drops_and_is_infinite_when_dark():
    lit = camera_bearing_variance(config.CAMERA_NOMINAL_MEAN)
    dim = camera_bearing_variance(config.CAMERA_NOMINAL_MEAN / 3)
    assert lit == pytest.approx(config.EKF_R_CAMERA_BEARING)
    assert dim == pytest.approx(9 * lit)
    assert math.isinf(camera_bearing_variance(config.CAMERA_DARK_MEAN - 1))
    assert math.isinf(camera_bearing_variance(None))


def test_bearing_update_corrects_heading_when_lit_and_is_skipped_in_the_dark():
    landmark = (2.0, 0.0)
    true_theta = 0.05                              # robot actually yawed 0.05 rad
    z = math.atan2(0.0, 2.0) - true_theta
    a = ekf()
    assert a.update_bearing(z, landmark, camera_bearing_variance(120.0))
    assert a.x[2] > 0.02                           # pulled towards the truth
    b = ekf()
    assert not b.update_bearing(z, landmark, camera_bearing_variance(5.0))
    assert b.x[2] == 0.0                           # lights off: no influence


def test_dim_camera_has_smaller_gain_than_lit_camera():
    z = -0.05
    lit, dim = ekf(), ekf()
    lit.update_bearing(z, (2.0, 0.0), camera_bearing_variance(120.0))
    dim.update_bearing(z, (2.0, 0.0), camera_bearing_variance(30.0))
    assert 0 < dim.x[2] < lit.x[2]


def test_outlier_bearing_is_gated():
    a = ekf()
    a.P = np.diag([1e-4, 1e-4, 1e-4])
    assert not a.update_bearing(1.0, (2.0, 0.0), camera_bearing_variance(120.0))
