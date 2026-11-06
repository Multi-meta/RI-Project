"""
test_traffic.py — Unit tests for traffic rules, one-way aisles, right-of-way,
reservation table, and deadlock resolution (Phase 15.5 - 15.8).
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import math
import pytest
from traffic import TrafficManager, AisleDirection, ReservationTable


def test_one_way_aisle_rules():
    tm = TrafficManager()
    
    # North aisle is eastbound (+X):
    # Moving from (-1.0, 2.5) to (1.0, 2.5) -> Valid (+x)
    assert tm.is_movement_allowed(-1.0, 2.5, 1.0, 2.5) is True
    # Moving from (1.0, 2.5) to (-1.0, 2.5) -> Rejected (-x)
    assert tm.is_movement_allowed(1.0, 2.5, -1.0, 2.5) is False
    
    # South aisle is westbound (-X):
    # Moving from (1.0, -2.5) to (-1.0, -2.5) -> Valid (-x)
    assert tm.is_movement_allowed(1.0, -2.5, -1.0, -2.5) is True
    # Moving from (-1.0, -2.5) to (1.0, -2.5) -> Rejected (+x)
    assert tm.is_movement_allowed(-1.0, -2.5, 1.0, -2.5) is False
    
    # Central corridor is bidirectional
    assert tm.is_movement_allowed(-2.0, 0.0, 2.0, 0.0) is True
    assert tm.is_movement_allowed(2.0, 0.0, -2.0, 0.0) is True


def test_right_of_way_evaluation():
    tm = TrafficManager()
    
    # 1. Carrying package overrides empty robot
    assert tm.evaluate_right_of_way(
        my_id="R2", my_pose=(0, 0, 0), my_carrying=True,
        other_id="R1", other_pose=(0.5, 0, 0), other_carrying=False
    ) is True
    
    assert tm.evaluate_right_of_way(
        my_id="R1", my_pose=(0, 0, 0), my_carrying=False,
        other_id="R2", other_pose=(0.5, 0, 0), other_carrying=True
    ) is False
    
    # 2. Main corridor has right-of-way over entering aisle
    assert tm.evaluate_right_of_way(
        my_id="R2", my_pose=(0, 0.5, 0), my_carrying=False,     # In corridor
        other_id="R1", other_pose=(0, 2.2, 0), other_carrying=False # In shelf aisle
    ) is True
    
    # 3. Tie-breaker by ID when states are equal
    assert tm.evaluate_right_of_way(
        my_id="R1", my_pose=(0, 0, 0), my_carrying=False,
        other_id="R2", other_pose=(0.5, 0, 0), other_carrying=False
    ) is True


def test_space_time_reservations():
    table = ReservationTable()
    
    # R1 reserves Bay SH1_B1 from t=10.0 to t=20.0
    ok = table.reserve("SH1_B1", "R1", start_time=10.0, duration=10.0)
    assert ok is True
    assert table.is_reserved("SH1_B1", current_time=15.0, querying_robot_id="R2") is True
    assert table.is_reserved("SH1_B1", current_time=15.0, querying_robot_id="R1") is False  # Own reservation
    
    # R2 tries overlapping reservation -> should fail
    conflict = table.reserve("SH1_B1", "R2", start_time=15.0, duration=10.0)
    assert conflict is False
    
    # R2 non-overlapping reservation (after t=20.0) -> should succeed
    ok_after = table.reserve("SH1_B1", "R2", start_time=21.0, duration=5.0)
    assert ok_after is True
    
    # Expire old reservations
    table.clean_expired(current_time=20.5)
    assert table.is_reserved("SH1_B1", current_time=20.5, querying_robot_id="R2") is False


def test_deadlock_detection_and_resolution():
    tm = TrafficManager(yield_dist=1.5, deadlock_timeout=4.0)
    
    # Two robots head-to-head, R1 has priority over R2
    r1_pose = (0.0, 0.0, 0.0)
    r2_pose = (0.8, 0.0, math.pi)
    
    # R1 should proceed
    action1, speed1 = tm.check_interaction("R1", r1_pose, False, "R2", r2_pose, False, now=10.0)
    assert action1 == "PROCEED"
    assert speed1 == 1.0
    
    # R2 must yield at now=10.0
    action2, speed2 = tm.check_interaction("R2", r2_pose, False, "R1", r1_pose, False, now=10.0)
    assert action2 == "YIELD"
    assert speed2 == 0.0
    
    # R2 still yielding before timeout (now=12.0)
    action2_mid, _ = tm.check_interaction("R2", r2_pose, False, "R1", r1_pose, False, now=12.0)
    assert action2_mid == "YIELD"
    
    # R2 deadlocked past timeout (now=15.0 > 10.0 + 4.0) -> triggers BACKOFF
    action2_deadlock, speed2_back = tm.check_interaction("R2", r2_pose, False, "R1", r1_pose, False, now=15.0)
    assert action2_deadlock == "DEADLOCK_BACKOFF"
    assert speed2_back < 0.0
    
    # R2 stuck even longer -> triggers REPLAN
    action2_replan, _ = tm.check_interaction("R2", r2_pose, False, "R1", r1_pose, False, now=18.0)
    assert action2_replan == "DEADLOCK_REPLAN"
