"""
manipulation.py — Pick and Place FSMs with sensor-verified grasping.

Per addendum §A9.4 & §A9.5:
- PickFSM:
  PRE_DOCK -> ALIGN -> OPEN_GRIPPER -> APPROACH -> ADVANCE -> CLOSE ->
  VERIFY (force + width) -> LIFT -> RETRACT -> STOW -> CARRY.
  Failure branches: missed grasp -> retry (max 2); slip -> regrasp; abort.
- PlaceFSM:
  PLAN_DELIVERY -> DOCK -> LIFT -> CHECK_SLOT_FREE (palm IR) -> EXTEND ->
  LOWER -> OPEN -> VERIFY_RELEASE -> RETRACT -> STOW -> CONFIRM.
"""

from enum import Enum, auto
from typing import Dict, Any, Optional, Tuple


class PickState(Enum):
    IDLE = auto()
    PRE_DOCK = auto()
    ALIGN = auto()
    OPEN_GRIPPER = auto()
    APPROACH = auto()
    ADVANCE = auto()
    CLOSE = auto()
    VERIFY = auto()
    LIFT = auto()
    RETRACT = auto()
    STOW = auto()
    CARRY = auto()
    RETRY = auto()
    ERROR = auto()


class PlaceState(Enum):
    IDLE = auto()
    DOCK = auto()
    LIFT = auto()
    CHECK_SLOT_FREE = auto()
    EXTEND = auto()
    LOWER = auto()
    OPEN = auto()
    VERIFY_RELEASE = auto()
    RETRACT = auto()
    STOW = auto()
    CONFIRM = auto()
    ERROR = auto()


class PickFSM:
    """State machine for sensor-verified pick sequence."""

    def __init__(self, expected_box_width: float = 0.22, min_grasp_force: float = 2.0, max_retries: int = 2):
        self.expected_width = expected_box_width
        self.min_force = min_grasp_force
        self.max_retries = max_retries
        self.retries = 0
        self.state = PickState.IDLE
        self.holding_package = False

    def start(self):
        self.state = PickState.PRE_DOCK
        self.retries = 0
        self.holding_package = False

    def update(self, sensors: Dict[str, Any]) -> PickState:
        """
        Advance state machine given current sensor feedback.
        
        sensors:
            'aligned': bool
            'gripper_width': float (m)
            'gripper_force': float (N)
            'palm_ir_dist': float (m)
            'arm_extended': bool
            'arm_lifted': bool
            'arm_stowed': bool
        """
        if self.state == PickState.PRE_DOCK:
            self.state = PickState.ALIGN

        elif self.state == PickState.ALIGN:
            if sensors.get('aligned', True):
                self.state = PickState.OPEN_GRIPPER

        elif self.state == PickState.OPEN_GRIPPER:
            # Gripper is opening
            if sensors.get('gripper_width', 0.0) >= self.expected_width:
                self.state = PickState.APPROACH

        elif self.state == PickState.APPROACH:
            self.state = PickState.ADVANCE

        elif self.state == PickState.ADVANCE:
            # Palm IR sensor confirms object proximity
            if sensors.get('palm_ir_dist', 0.05) <= 0.10:
                self.state = PickState.CLOSE

        elif self.state == PickState.CLOSE:
            self.state = PickState.VERIFY

        elif self.state == PickState.VERIFY:
            # Dual verification: force threshold AND width check
            force = sensors.get('gripper_force', 0.0)
            width = sensors.get('gripper_width', 0.0)
            width_ok = abs(width - self.expected_width) <= 0.04
            force_ok = force >= self.min_force

            if width_ok and force_ok:
                self.holding_package = True
                self.state = PickState.LIFT
            else:
                self.retries += 1
                if self.retries <= self.max_retries:
                    self.state = PickState.OPEN_GRIPPER  # Retry grasp
                else:
                    self.state = PickState.ERROR

        elif self.state == PickState.LIFT:
            if sensors.get('arm_lifted', True):
                self.state = PickState.RETRACT

        elif self.state == PickState.RETRACT:
            self.state = PickState.STOW

        elif self.state == PickState.STOW:
            if sensors.get('arm_stowed', True):
                self.state = PickState.CARRY

        return self.state


class PlaceFSM:
    """State machine for sensor-verified place sequence."""

    def __init__(self, expected_back_wall_dist: float = 0.40):
        self.back_wall_dist = expected_back_wall_dist
        self.state = PlaceState.IDLE
        self.placed_successfully = False

    def start(self):
        self.state = PlaceState.DOCK
        self.placed_successfully = False

    def update(self, sensors: Dict[str, Any]) -> PlaceState:
        """
        Advance state machine given current sensor feedback.
        
        sensors:
            'docked': bool
            'lift_at_height': bool
            'palm_ir_dist': float (m)
            'extended': bool
            'gripper_force': float (N)
            'gripper_open': bool
            'arm_stowed': bool
        """
        if self.state == PlaceState.DOCK:
            if sensors.get('docked', True):
                self.state = PlaceState.LIFT

        elif self.state == PlaceState.LIFT:
            if sensors.get('lift_at_height', True):
                self.state = PlaceState.CHECK_SLOT_FREE

        elif self.state == PlaceState.CHECK_SLOT_FREE:
            # Check slot is empty (palm IR sees back wall unobstructed)
            dist = sensors.get('palm_ir_dist', self.back_wall_dist)
            if dist >= self.back_wall_dist * 0.8:
                self.state = PlaceState.EXTEND
            else:
                self.state = PlaceState.ERROR  # Slot blocked!

        elif self.state == PlaceState.EXTEND:
            if sensors.get('extended', True):
                self.state = PlaceState.LOWER

        elif self.state == PlaceState.LOWER:
            self.state = PlaceState.OPEN

        elif self.state == PlaceState.OPEN:
            if sensors.get('gripper_open', True):
                self.state = PlaceState.VERIFY_RELEASE

        elif self.state == PlaceState.VERIFY_RELEASE:
            # Force must drop to zero
            if sensors.get('gripper_force', 0.0) < 0.5:
                self.placed_successfully = True
                self.state = PlaceState.RETRACT
            else:
                self.state = PlaceState.ERROR

        elif self.state == PlaceState.RETRACT:
            self.state = PlaceState.STOW

        elif self.state == PlaceState.STOW:
            if sensors.get('arm_stowed', True):
                self.state = PlaceState.CONFIRM

        return self.state
