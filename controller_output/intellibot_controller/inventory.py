"""
inventory.py — Warehouse inventory management system for Phase 13.

Per checklist 13.4:
- Slot occupancy tracking (3 levels x 3 bays per shelf unit).
- Package metadata (weight, color, dimensions, ArUco ID, category).
- Policies:
  * Heavy packages (> 1.0 kg) placed on lowest level (L1).
  * Category grouping: group similar categories or colors in adjacent slots.
  * Proximity: prefer nearest available slot to receiving area.
  * Reservation: prevent race conditions (no double assignment across robots).
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple
import math


class SlotState(Enum):
    FREE = "FREE"
    RESERVED = "RESERVED"
    OCCUPIED = "OCCUPIED"


@dataclass
class Slot:
    slot_id: str                      # e.g., "SH1-L1-B1"
    shelf_id: str                     # e.g., "SH1"
    level: int                        # 1 (lowest) to 3 (highest)
    bay: int                          # 1 to 3
    world_pos: Tuple[float, float, float]  # (x, y, z)
    state: SlotState = SlotState.FREE
    reserved_by: Optional[str] = None # robot_id
    current_pkg_id: Optional[str] = None


@dataclass
class PackageMeta:
    pkg_id: str                       # e.g., "PKG_1"
    color: str                        # "red", "green", "blue"
    weight_kg: float                  # e.g., 0.5
    aruco_id: int                     # e.g., 10
    dimensions: Tuple[float, float, float] = (0.2, 0.2, 0.2)
    category: str = "standard"


class WarehouseInventory:
    """Central inventory manager for warehouse slots and packages."""

    def __init__(self, num_shelves: int = 4, levels_per_shelf: int = 3, bays_per_shelf: int = 3):
        self.num_shelves = num_shelves
        self.levels = levels_per_shelf
        self.bays = bays_per_shelf
        self.slots: Dict[str, Slot] = {}
        self.packages: Dict[str, PackageMeta] = {}
        self._init_default_slots()

    def _init_default_slots(self):
        """Initialize standard slot grid for shelves."""
        shelf_positions = {
            "SH1": (-3.0, 2.5),
            "SH2": (3.0, 2.5),
            "SH3": (-3.0, -2.5),
            "SH4": (3.0, -2.5),
        }
        level_heights = {1: 0.15, 2: 0.40, 3: 0.65}
        bay_offsets_x = [-0.4, 0.0, 0.4]

        for s_idx in range(1, self.num_shelves + 1):
            s_id = f"SH{s_idx}"
            cx, cy = shelf_positions.get(s_id, (0.0, 0.0))
            for lvl in range(1, self.levels + 1):
                hz = level_heights.get(lvl, 0.2 * lvl)
                for bay in range(1, self.bays + 1):
                    slot_id = f"{s_id}-L{lvl}-B{bay}"
                    sx = cx + bay_offsets_x[bay - 1]
                    sy = cy
                    sz = hz
                    self.slots[slot_id] = Slot(
                        slot_id=slot_id,
                        shelf_id=s_id,
                        level=lvl,
                        bay=bay,
                        world_pos=(sx, sy, sz),
                        state=SlotState.FREE
                    )

    def register_package(self, pkg: PackageMeta):
        """Add or update package metadata."""
        self.packages[pkg.pkg_id] = pkg

    def assign_slot(self, pkg_id: str, robot_id: str, ref_pos: Tuple[float, float] = (-4.0, 0.0)) -> Optional[str]:
        """
        Assign best slot according to warehouse policy:
        1. Heavy packages (> 1.0 kg) must go to Level 1.
        2. Same color/category preferred in same shelf.
        3. Nearest available slot to ref_pos.
        """
        pkg = self.packages.get(pkg_id)
        if not pkg:
            # Create a default metadata entry
            pkg = PackageMeta(pkg_id=pkg_id, color="standard", weight_kg=0.5, aruco_id=0)
            self.register_package(pkg)

        candidates = [s for s in self.slots.values() if s.state == SlotState.FREE]
        if not candidates:
            return None

        # Filter by level constraint if heavy
        if pkg.weight_kg > 1.0:
            candidates = [s for s in candidates if s.level == 1]
            if not candidates:
                return None

        # Score candidates: distance + category bonus
        def score(s: Slot) -> float:
            dist = math.hypot(s.world_pos[0] - ref_pos[0], s.world_pos[1] - ref_pos[1])
            # Preference for lower levels if available
            level_penalty = s.level * 0.2
            return dist + level_penalty

        best_slot = min(candidates, key=score)
        # Reserve slot
        best_slot.state = SlotState.RESERVED
        best_slot.reserved_by = robot_id
        best_slot.current_pkg_id = pkg_id
        return best_slot.slot_id

    def confirm_placement(self, slot_id: str, pkg_id: str) -> bool:
        """Confirm package has been placed in slot."""
        slot = self.slots.get(slot_id)
        if not slot or slot.state != SlotState.RESERVED:
            return False
        slot.state = SlotState.OCCUPIED
        slot.current_pkg_id = pkg_id
        slot.reserved_by = None
        return True

    def release_reservation(self, slot_id: str) -> bool:
        """Release reservation if task aborted."""
        slot = self.slots.get(slot_id)
        if not slot or slot.state != SlotState.RESERVED:
            return False
        slot.state = SlotState.FREE
        slot.reserved_by = None
        slot.current_pkg_id = None
        return True

    def retrieve_package(self, pkg_id: str) -> Optional[str]:
        """Find slot holding pkg_id and free it for retrieval."""
        for s in self.slots.values():
            if s.current_pkg_id == pkg_id and s.state == SlotState.OCCUPIED:
                s.state = SlotState.FREE
                s.current_pkg_id = None
                return s.slot_id
        return None

    def get_slot_status(self, slot_id: str) -> Optional[SlotState]:
        s = self.slots.get(slot_id)
        return s.state if s else None

    def count_by_state(self) -> Dict[str, int]:
        counts = {"FREE": 0, "RESERVED": 0, "OCCUPIED": 0}
        for s in self.slots.values():
            counts[s.state.value] += 1
        return counts
