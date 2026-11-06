"""
test_fleet_manager.py — Unit tests for FleetManagerCore: task allocation,
reassignment, inventory confirmation, and multi-robot coordination (Phase 15.2 - 15.4).
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'fleet_manager'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import pytest
from fleet_manager import FleetManagerCore, FleetTask
from inventory import WarehouseInventory, SlotState


def test_greedy_nearest_allocation():
    core = FleetManagerCore()
    # R1 at (-4.8, 2.5), R2 at (4.0, 0.0)
    core.update_robot_heartbeat('R1', (-4.8, 2.5, 0), (0, 0), 'IDLE', None, now=1.0)
    core.update_robot_heartbeat('R2', (4.0, 0.0, 0), (0, 0), 'IDLE', None, now=1.0)
    
    # Task A near R1 (Receiving pile)
    task_a = FleetTask("T_A", "STORE", "pkg_1", (-4.5, 2.0, 0.2), (0.0, 2.0, 0.5))
    # Task B near R2 (Dispatch area)
    task_b = FleetTask("T_B", "STORE", "pkg_2", (3.8, 0.5, 0.2), (1.0, -2.0, 0.5))
    
    core.add_task(task_a)
    core.add_task(task_b)
    
    assignments = core.allocate_pending_tasks(now=2.0)
    assert len(assignments) == 2
    
    # Task A should go to R1 (distance ~0.5m vs ~8.5m)
    assert core.tasks["T_A"].assigned_robot == 'R1'
    # Task B should go to R2 (distance ~0.5m vs ~8.8m)
    assert core.tasks["T_B"].assigned_robot == 'R2'


def test_no_double_assignment_to_busy_robot():
    core = FleetManagerCore()
    core.update_robot_heartbeat('R1', (0, 0, 0), (0, 0), 'IDLE', None, now=1.0)
    core.update_robot_heartbeat('R2', (10, 10, 0), (0, 0), 'BUSY', None, now=1.0)
    
    t1 = FleetTask("T1", "STORE", "pkg_1", (0, 0, 0), (1, 1, 0))
    t2 = FleetTask("T2", "STORE", "pkg_2", (0, 0, 0), (2, 2, 0))
    core.add_task(t1)
    core.add_task(t2)
    
    # Only R1 is idle, so only 1 task can be assigned
    assignments = core.allocate_pending_tasks(now=2.0)
    assert len(assignments) == 1
    assert assignments[0] == ("T1", "R1")
    assert core.tasks["T2"].status == 'PENDING'
    assert core.tasks["T2"].assigned_robot is None


def test_task_lifecycle_and_inventory_confirmation():
    inv = WarehouseInventory()
    slot_id = inv.assign_slot("pkg_1", robot_id='R1')
    assert slot_id is not None
    slot = inv.slots[slot_id]
    assert slot.state == SlotState.RESERVED
    
    core = FleetManagerCore(inventory=inv)
    core.update_robot_heartbeat('R1', (0, 0, 0), (0, 0), 'IDLE', None, now=1.0)
    
    task = FleetTask("T1", "STORE", "pkg_1", (0, 0, 0), slot.world_pos, slot_id=slot_id)
    core.add_task(task)
    
    core.allocate_pending_tasks(now=2.0)
    assert task.status == 'ASSIGNED'
    
    core.handle_robot_response('R1', 'T1', 'accept')
    assert task.status == 'IN_PROGRESS'
    
    core.handle_robot_response('R1', 'T1', 'complete')
    assert task.status == 'COMPLETED'
    assert "T1" in core.completed_tasks
    # Inventory slot should now be OCCUPIED
    assert slot.state == SlotState.OCCUPIED


def test_task_timeout_and_reassignment():
    core = FleetManagerCore()
    core.update_robot_heartbeat('R1', (0, 0, 0), (0, 0), 'IDLE', None, now=1.0)
    
    task = FleetTask("T1", "STORE", "pkg_1", (0, 0, 0), (1, 1, 0), timeout=10.0)
    core.add_task(task)
    
    core.allocate_pending_tasks(now=2.0)
    assert task.assigned_robot == 'R1'
    assert task.status == 'ASSIGNED'
    
    # 5 seconds elapsed -> not timed out
    timed_out = core.check_timeouts(now=7.0)
    assert len(timed_out) == 0
    assert task.status == 'ASSIGNED'
    
    # 15 seconds elapsed -> timed out (> 10s)
    timed_out = core.check_timeouts(now=18.0)
    assert len(timed_out) == 1
    assert timed_out[0] == "T1"
    assert task.status == 'PENDING'
    assert task.assigned_robot is None
