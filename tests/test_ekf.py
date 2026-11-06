"""
test_ekf.py — Unit tests for EKFLocalizer.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import pytest
import numpy as np
from ekf_localization import EKFLocalizer


def test_ekf_predict():
    x0 = np.array([0.0, 0.0, 0.0])
    P0 = np.diag([0.1**2, 0.1**2, (5*np.pi/180)**2])
    Q = np.diag([0.02**2, 0.02**2, (2*np.pi/180)**2])
    
    ekf = EKFLocalizer(x0, P0, Q)
    ekf.predict(ds=1.0, dtheta=0.0)
    x, y, theta = ekf.get_pose()
    assert abs(x - 1.0) < 1e-4
    assert abs(y) < 1e-4


def test_ekf_gps_update_and_gating():
    x0 = np.array([1.0, 0.0, 0.0])
    P0 = np.diag([0.1**2, 0.1**2, (5*np.pi/180)**2])
    Q = np.diag([0.02**2, 0.02**2, (2*np.pi/180)**2])
    
    ekf = EKFLocalizer(x0, P0, Q)
    R_gps = np.diag([0.1**2, 0.1**2])
    
    # Valid update
    accepted = ekf.update_gps(np.array([1.05, 0.02]), R_gps)
    assert accepted
    
    # Outlier update rejected by gating
    rejected = ekf.update_gps(np.array([100.0, 200.0]), R_gps, gate_threshold=9.21)
    assert not rejected
