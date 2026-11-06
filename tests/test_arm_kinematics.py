"""
test_arm_kinematics.py — Unit tests for arm kinematics (Phase 14.3).
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import pytest
import math
import numpy as np
from arm_kinematics import ArmKinematics


def test_ik_fk_round_trip():
    arm = ArmKinematics()
    
    # Test multiple reachable points in workspace
    test_points = [
        (0.35, 0.10, 0.50, 0.0),
        (0.40, -0.15, 0.20, 0.2),
        (0.25, 0.25, 0.80, -0.3),
        (0.45, 0.00, 1.00, 0.0),
    ]
    
    for tx, ty, tz, tyaw in test_points:
        sol = arm.inverse_kinematics(tx, ty, tz, target_yaw=tyaw, elbow_up=False)
        assert sol is not None, f"Point ({tx}, {ty}, {tz}) should be reachable"
        
        lift_z, theta1, theta2, theta3 = sol
        fx, fy, fz, fyaw = arm.forward_kinematics(lift_z, theta1, theta2, theta3)
        
        # Round-trip precision must be < 1e-6
        assert math.hypot(fx - tx, fy - ty) < 1e-6
        assert abs(fz - tz) < 1e-6
        assert abs(fyaw - tyaw) < 1e-6


def test_both_elbow_solutions():
    arm = ArmKinematics()
    sols = arm.get_both_solutions(0.35, 0.0, 0.5, target_yaw=0.0)
    assert len(sols) == 2
    # One has positive theta2, other negative
    assert sols[0][2] * sols[1][2] < 0


def test_out_of_reach_rejection():
    arm = ArmKinematics()
    # Beyond max reach (0.30 + 0.28 = 0.58m)
    assert arm.inverse_kinematics(0.70, 0.0, 0.5) is None
    # Below min reach (|0.30 - 0.28| = 0.02m)
    assert arm.inverse_kinematics(0.01, 0.0, 0.5) is None
    # Beyond lift limits
    assert arm.inverse_kinematics(0.35, 0.0, 1.5) is None
    assert arm.inverse_kinematics(0.35, 0.0, -0.2) is None


def test_shelf_workspace_coverage():
    """Verify that arm reaches heights for all 3 shelf levels (0.15m, 0.55m, 0.95m)."""
    arm = ArmKinematics()
    shelf_heights = [0.15, 0.55, 0.95]
    reach_dist = 0.40  # 40 cm standoff from shelf slot
    
    for h in shelf_heights:
        sol = arm.inverse_kinematics(reach_dist, 0.0, h, target_yaw=0.0)
        assert sol is not None, f"Shelf level at height {h}m must be reachable"
