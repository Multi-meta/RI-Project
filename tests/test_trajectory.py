"""
test_trajectory.py — Unit tests for quintic trajectory generation (Phase 14.4).
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import pytest
import numpy as np
from trajectory import quintic_profile, QuinticTrajectory


def test_quintic_boundary_conditions():
    s0, s_dot0, s_ddot0 = quintic_profile(0.0)
    assert s0 == 0.0
    assert abs(s_dot0) < 1e-9
    assert abs(s_ddot0) < 1e-9

    s1, s_dot1, s_ddot1 = quintic_profile(1.0)
    assert s1 == 1.0
    assert abs(s_dot1) < 1e-9
    assert abs(s_ddot1) < 1e-9


def test_trajectory_sampling_and_limits():
    q0 = np.array([0.0, 0.0, 0.0, 0.0])
    q1 = np.array([0.5, 1.0, -0.5, 0.2])
    max_v = 1.0
    max_a = 2.0
    
    traj = QuinticTrajectory(q0, q1, max_velocity=max_v, max_acceleration=max_a)
    assert traj.T > 0.0
    
    # Sample along trajectory
    times = np.linspace(0.0, traj.T, 50)
    for t in times:
        pos, vel, acc = traj.sample(t)
        # Check velocity limit
        assert np.all(np.abs(vel) <= max_v + 1e-4)
        # Check acceleration limit
        assert np.all(np.abs(acc) <= max_a + 1e-4)
        
    # Check start and end poses
    p_start, v_start, a_start = traj.sample(0.0)
    assert np.allclose(p_start, q0, atol=1e-6)
    assert np.allclose(v_start, 0.0, atol=1e-6)
    
    p_end, v_end, a_end = traj.sample(traj.T)
    assert np.allclose(p_end, q1, atol=1e-6)
    assert np.allclose(v_end, 0.0, atol=1e-6)
