"""
traffic.py — Traffic rules, right-of-way, aisle reservation, and deadlock resolution.

Per checklist Phase 15 (§15.5 - 15.8):
- One-way aisle definitions & direction verification.
- Priority & Right-of-Way rules at intersections.
- Space-time reservation table for multi-robot coordination.
- Deadlock detection and resolution (back-off / alternate route).
"""

import math
import time
from enum import Enum
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, field


class AisleDirection(Enum):
    EASTBOUND = (1.0, 0.0)      # +X direction
    WESTBOUND = (-1.0, 0.0)     # -X direction
    NORTHBOUND = (0.0, 1.0)     # +Y direction
    SOUTHBOUND = (0.0, -1.0)    # -Y direction
    BIDIRECTIONAL = (0.0, 0.0)  # Either direction allowed


@dataclass
class Aisle:
    """Represents a corridor or aisle with designated direction."""
    name: str
    x_bounds: Tuple[float, float]  # (xmin, xmax)
    y_bounds: Tuple[float, float]  # (ymin, ymax)
    direction: AisleDirection
    
    def contains(self, x: float, y: float) -> bool:
        return (self.x_bounds[0] <= x <= self.x_bounds[1] and
                self.y_bounds[0] <= y <= self.y_bounds[1])

    def violates_direction(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        """Returns True if movement from (x1, y1) to (x2, y2) violates one-way rule."""
        if not (self.contains(x1, y1) or self.contains(x2, y2)):
            return False
        if self.direction == AisleDirection.BIDIRECTIONAL:
            return False
            
        dx = x2 - x1
        dy = y2 - y1
        dir_x, dir_y = self.direction.value
        dot = dx * dir_x + dy * dir_y
        # If moving substantially in opposite direction of one-way flow
        return dot < -0.05


@dataclass
class Reservation:
    zone_id: str
    robot_id: str
    start_time: float
    end_time: float


class ReservationTable:
    """Space-time reservation table for warehouse zones, aisles, and docks."""
    
    def __init__(self):
        self.reservations: List[Reservation] = []

    def clean_expired(self, current_time: float):
        self.reservations = [r for r in self.reservations if r.end_time > current_time]

    def reserve(self, zone_id: str, robot_id: str, start_time: float, duration: float) -> bool:
        end_time = start_time + duration
        # Check collision with other reservations
        for r in self.reservations:
            if r.zone_id == zone_id and r.robot_id != robot_id:
                # Time overlap check: start_time < r.end_time and end_time > r.start_time
                if start_time < r.end_time and end_time > r.start_time:
                    return False  # Conflict
                    
        self.reservations.append(Reservation(zone_id, robot_id, start_time, end_time))
        return True

    def is_reserved(self, zone_id: str, current_time: float, querying_robot_id: str) -> bool:
        for r in self.reservations:
            if r.zone_id == zone_id and r.robot_id != querying_robot_id:
                if r.start_time <= current_time <= r.end_time:
                    return True
        return False

    def cancel(self, zone_id: str, robot_id: str):
        self.reservations = [r for r in self.reservations if not (r.zone_id == zone_id and r.robot_id == robot_id)]


class TrafficManager:
    """
    Evaluates traffic rules, intersection right-of-way, yielding, and deadlocks.
    """

    def __init__(self, yield_dist: float = 1.2, deadlock_timeout: float = 5.0):
        self.yield_dist = yield_dist
        self.deadlock_timeout = deadlock_timeout
        self.reservations = ReservationTable()
        
        # Define Warehouse v2 standard aisles
        self.aisles: List[Aisle] = [
            # North Shelf Aisle (between North shelves and wall) -> Eastbound
            Aisle("aisle_north", (-4.0, 4.0), (1.8, 3.2), AisleDirection.EASTBOUND),
            # South Shelf Aisle (between South shelves and wall) -> Westbound
            Aisle("aisle_south", (-4.0, 4.0), (-3.2, -1.8), AisleDirection.WESTBOUND),
            # Central Main Corridor -> Bidirectional
            Aisle("corridor_center", (-4.5, 4.5), (-1.2, 1.2), AisleDirection.BIDIRECTIONAL),
        ]
        
        # Track yield start times for deadlock detection: robot_id -> start_time
        self.yield_start_times: Dict[str, float] = {}

    def is_movement_allowed(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        """Check if trajectory segment respects all one-way aisle rules."""
        for aisle in self.aisles:
            if aisle.violates_direction(x1, y1, x2, y2):
                return False
        return True

    def evaluate_right_of_way(self, my_id: str, my_pose: Tuple[float, float, float],
                              my_carrying: bool, other_id: str,
                              other_pose: Tuple[float, float, float],
                              other_carrying: bool) -> bool:
        """
        Determines whether 'my_id' has right-of-way over 'other_id'.
        Returns True if 'my_id' has priority (other robot must yield).
        Returns False if 'my_id' must yield to 'other_id'.
        """
        # Rule 1: Loaded robot carrying package has priority over empty robot
        if my_carrying and not other_carrying:
            return True
        if not my_carrying and other_carrying:
            return False

        # Rule 2: Robot already moving in central corridor has priority over robot entering
        my_in_corridor = abs(my_pose[1]) <= 1.2
        other_in_corridor = abs(other_pose[1]) <= 1.2
        if my_in_corridor and not other_in_corridor:
            return True
        if not my_in_corridor and other_in_corridor:
            return False

        # Rule 3: Tie-breaker by alphanumeric ID (e.g. 'R1' > 'R2' or 'ROBOT_1' > 'ROBOT_2')
        return my_id < other_id

    def check_interaction(self, my_id: str, my_pose: Tuple[float, float, float],
                          my_carrying: bool, other_id: str,
                          other_pose: Tuple[float, float, float],
                          other_carrying: bool,
                          now: float) -> Tuple[str, float]:
        """
        Checks spatial interaction between two robots.
        
        Returns:
            (action, speed_cap):
            - ("PROCEED", 1.0): No conflict or has priority.
            - ("YIELD", 0.0): Must yield to other robot.
            - ("DEADLOCK_REPLAN", 0.0): Deadlock detected; must replan alternate route.
            - ("DEADLOCK_BACKOFF", -0.15): Deadlock detected; back off.
        """
        mx, my, _ = my_pose
        ox, oy, _ = other_pose
        dist = math.hypot(mx - ox, my - oy)

        if dist > self.yield_dist:
            # Out of yield range; reset yield timer if any
            if my_id in self.yield_start_times:
                del self.yield_start_times[my_id]
            return ("PROCEED", 1.0)

        # In interaction zone: evaluate priority
        i_have_priority = self.evaluate_right_of_way(
            my_id, my_pose, my_carrying, other_id, other_pose, other_carrying
        )

        if i_have_priority:
            if my_id in self.yield_start_times:
                del self.yield_start_times[my_id]
            return ("PROCEED", 1.0)

        # Must yield: check for deadlock timeout
        if my_id not in self.yield_start_times:
            self.yield_start_times[my_id] = now
            return ("YIELD", 0.0)

        wait_time = now - self.yield_start_times[my_id]
        if wait_time > self.deadlock_timeout:
            # Deadlock resolution: lower-priority robot backs up or replans
            if wait_time > self.deadlock_timeout + 3.0:
                return ("DEADLOCK_REPLAN", 0.0)
            return ("DEADLOCK_BACKOFF", -0.15)

        return ("YIELD", 0.0)
