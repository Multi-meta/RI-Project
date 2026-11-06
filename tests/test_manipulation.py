"""
test_manipulation.py — Unit tests for PickFSM and PlaceFSM (Phase 14.6 - 14.8).
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import pytest
from manipulation import PickFSM, PickState, PlaceFSM, PlaceState


def test_pick_fsm_nominal():
    fsm = PickFSM(expected_box_width=0.22, min_grasp_force=2.0)
    fsm.start()
    assert fsm.state == PickState.PRE_DOCK
    
    # Sensors reporting nominal grasp conditions
    nominal_sensors = {
        'aligned': True,
        'gripper_width': 0.22,
        'gripper_force': 3.5,
        'palm_ir_dist': 0.05,
        'arm_lifted': True,
        'arm_stowed': True
    }
    
    # Step through sequence
    states = [fsm.update(nominal_sensors) for _ in range(10)]
    assert fsm.state == PickState.CARRY
    assert fsm.holding_package is True


def test_pick_fsm_retry_on_miss():
    fsm = PickFSM(expected_box_width=0.22, min_grasp_force=2.0, max_retries=2)
    fsm.start()
    
    # 1. Open gripper and advance
    fsm.update({'aligned': True, 'gripper_width': 0.25, 'palm_ir_dist': 0.05})
    fsm.update({'aligned': True, 'gripper_width': 0.25, 'palm_ir_dist': 0.05})
    fsm.update({'aligned': True, 'gripper_width': 0.25, 'palm_ir_dist': 0.05})
    fsm.update({'aligned': True, 'gripper_width': 0.25, 'palm_ir_dist': 0.05})
    
    # 2. Close with missed grasp (too narrow, low force)
    fsm.update({'aligned': True, 'gripper_width': 0.05, 'gripper_force': 0.2})
    fsm.update({'aligned': True, 'gripper_width': 0.05, 'gripper_force': 0.2})
    fsm.update({'aligned': True, 'gripper_width': 0.05, 'gripper_force': 0.2})

    # Should have retried and gone back to OPEN_GRIPPER
    assert fsm.retries == 1
    assert fsm.state == PickState.OPEN_GRIPPER
    
    # 3. Now provide good grasp
    sensors_good = {'aligned': True, 'gripper_width': 0.22, 'gripper_force': 3.0, 'palm_ir_dist': 0.05, 'arm_lifted': True, 'arm_stowed': True}
    fsm.update({'aligned': True, 'gripper_width': 0.25, 'palm_ir_dist': 0.05})
    for _ in range(8):
        fsm.update(sensors_good)
    
    assert fsm.state == PickState.CARRY
    assert fsm.holding_package is True


def test_place_fsm_nominal_and_occupied():
    # Nominal
    place = PlaceFSM(expected_back_wall_dist=0.40)
    place.start()
    sensors_nominal = {
        'docked': True,
        'lift_at_height': True,
        'palm_ir_dist': 0.38,  # Slot is clear
        'extended': True,
        'gripper_open': True,
        'gripper_force': 0.0,
        'arm_stowed': True
    }
    for _ in range(10):
        place.update(sensors_nominal)
    assert place.state == PlaceState.CONFIRM
    assert place.placed_successfully is True

    # Slot occupied
    place_blocked = PlaceFSM(expected_back_wall_dist=0.40)
    place_blocked.start()
    sensors_blocked = {
        'docked': True,
        'lift_at_height': True,
        'palm_ir_dist': 0.10,  # Object already inside!
    }
    for _ in range(4):
        place_blocked.update(sensors_blocked)
    assert place_blocked.state == PlaceState.ERROR
