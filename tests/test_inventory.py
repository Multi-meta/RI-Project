"""
test_inventory.py — Unit tests for WarehouseInventory system (Phase 13.4).
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import pytest
from inventory import WarehouseInventory, SlotState, PackageMeta


def test_inventory_init_and_capacity():
    inv = WarehouseInventory(num_shelves=4, levels_per_shelf=3, bays_per_shelf=3)
    # 4 * 3 * 3 = 36 slots
    assert len(inv.slots) == 36
    counts = inv.count_by_state()
    assert counts["FREE"] == 36
    assert counts["RESERVED"] == 0
    assert counts["OCCUPIED"] == 0


def test_heavy_package_policy():
    inv = WarehouseInventory()
    heavy_pkg = PackageMeta(pkg_id="HEAVY_1", color="red", weight_kg=2.5, aruco_id=1)
    inv.register_package(heavy_pkg)
    
    assigned_slot = inv.assign_slot("HEAVY_1", robot_id="R1")
    assert assigned_slot is not None
    # Must be on Level 1
    assert "-L1-" in assigned_slot
    assert inv.get_slot_status(assigned_slot) == SlotState.RESERVED


def test_reservation_and_confirmation_cycle():
    inv = WarehouseInventory()
    pkg = PackageMeta(pkg_id="PKG_A", color="blue", weight_kg=0.5, aruco_id=2)
    inv.register_package(pkg)
    
    slot_id = inv.assign_slot("PKG_A", robot_id="R1")
    assert inv.get_slot_status(slot_id) == SlotState.RESERVED
    
    # Confirm placement
    assert inv.confirm_placement(slot_id, "PKG_A")
    assert inv.get_slot_status(slot_id) == SlotState.OCCUPIED
    
    # Retrieve
    retrieved_slot = inv.retrieve_package("PKG_A")
    assert retrieved_slot == slot_id
    assert inv.get_slot_status(slot_id) == SlotState.FREE


def test_no_double_assignment():
    """Property test: two robots asking for slots cannot be given the same slot."""
    inv = WarehouseInventory(num_shelves=1, levels_per_shelf=1, bays_per_shelf=2)
    inv.register_package(PackageMeta(pkg_id="P1", color="red", weight_kg=0.5, aruco_id=1))
    inv.register_package(PackageMeta(pkg_id="P2", color="green", weight_kg=0.5, aruco_id=2))
    inv.register_package(PackageMeta(pkg_id="P3", color="blue", weight_kg=0.5, aruco_id=3))
    
    s1 = inv.assign_slot("P1", robot_id="R1")
    s2 = inv.assign_slot("P2", robot_id="R2")
    s3 = inv.assign_slot("P3", robot_id="R1")
    
    assert s1 != s2
    assert s1 is not None and s2 is not None
    assert s3 is None  # Capacity 2 exhausted!
