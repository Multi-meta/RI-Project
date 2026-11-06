"""
test_recovery.py — Unit tests for recovery behaviors FSM.
"""
import sys
import os
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import pytest
from recovery_behaviors import RecoveryBehavior, RecoveryConfig, RecoveryState


def test_wait_clears():
    cfg = RecoveryConfig(wait_timeout=2.0)
    rec = RecoveryBehavior(cfg)
    rec.trigger()
    assert rec.get_state() == RecoveryState.WAIT
    
    # Obstacle present
    cmd = rec.update(0.1, front_clearance=0.1, rear_clearance=1.0, current_heading=0.0, current_pos=(0, 0))
    assert cmd == (0.0, 0.0)
    assert rec.is_active()
    
    # Obstacle clears
    cmd = rec.update(0.1, front_clearance=0.5, rear_clearance=1.0, current_heading=0.0, current_pos=(0, 0))
    assert rec.get_state() == RecoveryState.IDLE
    assert rec.succeeded()


def test_rotate_finds_clearance():
    cfg = RecoveryConfig(wait_timeout=0.05)
    rec = RecoveryBehavior(cfg)
    rec._enter_state(RecoveryState.ROTATE)
    rec.rotate_start_heading = 0.0
    
    cmd = rec.update(0.1, front_clearance=0.5, rear_clearance=1.0, current_heading=0.3, current_pos=(0, 0))
    assert rec.get_state() == RecoveryState.REPLAN


def test_backup_and_replan():
    cfg = RecoveryConfig(backup_distance=0.3)
    rec = RecoveryBehavior(cfg)
    rec._enter_state(RecoveryState.BACKUP)
    rec.backup_start_pos = (0.0, 0.0)
    
    # Backing up
    cmd = rec.update(0.1, front_clearance=0.1, rear_clearance=1.0, current_heading=0.0, current_pos=(-0.1, 0.0))
    assert cmd[0] < 0.0
    
    # Finished backing up
    cmd = rec.update(0.1, front_clearance=0.1, rear_clearance=1.0, current_heading=0.0, current_pos=(-0.35, 0.0))
    assert rec.get_state() == RecoveryState.REPLAN
