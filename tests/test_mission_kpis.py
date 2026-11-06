"""
test_mission_kpis.py — Unit tests for KPI tracking and reporting.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import pytest
from mission_kpis import MissionKPITracker, ReplanReason


def test_mission_kpis_tracking():
    tracker = MissionKPITracker("R1")
    tracker.update_position(0.0, 0.0, 10.0)
    tracker.update_position(3.0, 4.0, 15.0)  # moved 5m
    assert abs(tracker.kpis.path_length - 5.0) < 0.01
    
    tracker.update_clearance(0.25, 12.0)
    assert tracker.kpis.min_clearance == 0.25
    
    tracker.record_collision(is_front=True)
    assert tracker.kpis.bumper_collisions == 1
    assert tracker.kpis.front_bumper_hits == 1
    
    tracker.record_replan(ReplanReason.BLOCKED_PATH)
    assert tracker.kpis.replan_count == 1
    
    rep = tracker.generate_report()
    assert rep['robot_id'] == "R1"
    assert rep['kpis']['bumper_collisions'] == 1
