"""
arm_kinematics.py — Kinematics and workspace analysis for Robot v2 mobile manipulator.

Per addendum §A9.3 & checklist 14.3:
- Planar 2R arm + lift (z) + wrist yaw:
  * Link 1 length: l1 = 0.30 m
  * Link 2 length: l2 = 0.28 m
  * Reach: up to 0.58 m
  * Lift range: 0.0 to 1.1 m
- Analytic IK:
  * Both elbow-up and elbow-down solutions
  * Joint limit checking (shoulder +-120 deg, elbow +-120 deg)
  * Out-of-reach rejection
- Forward kinematics (FK):
  * FK(lift_z, theta1, theta2, theta3) -> (x, y, z, yaw)
- Workspace coverage analysis for shelves and receiving pile.
"""

import math
from typing import Optional, Tuple, List


class ArmKinematics:
    """Analytical FK and IK for lift + 2R arm + wrist yaw."""

    def __init__(self, l1: float = 0.30, l2: float = 0.28,
                 lift_min: float = 0.0, lift_max: float = 1.1,
                 theta1_limits: Tuple[float, float] = (-2.094, 2.094),   # +-120 deg
                 theta2_limits: Tuple[float, float] = (-2.094, 2.094),   # +-120 deg
                 wrist_limits: Tuple[float, float] = (-3.14159, 3.14159)):
        self.l1 = l1
        self.l2 = l2
        self.lift_min = lift_min
        self.lift_max = lift_max
        self.theta1_limits = theta1_limits
        self.theta2_limits = theta2_limits
        self.wrist_limits = wrist_limits

    def forward_kinematics(self, lift_z: float, theta1: float, theta2: float, theta3: float) -> Tuple[float, float, float, float]:
        """
        Forward kinematics from joint angles to end-effector pose in arm-base frame.
        
        Returns:
            (x, y, z, yaw)
        """
        x = self.l1 * math.cos(theta1) + self.l2 * math.cos(theta1 + theta2)
        y = self.l1 * math.sin(theta1) + self.l2 * math.sin(theta1 + theta2)
        z = lift_z
        yaw = theta1 + theta2 + theta3
        return x, y, z, yaw

    def inverse_kinematics(self, target_x: float, target_y: float, target_z: float,
                           target_yaw: float = 0.0,
                           elbow_up: bool = False) -> Optional[Tuple[float, float, float, float]]:
        """
        Analytic inverse kinematics.
        
        Args:
            target_x, target_y: planar reach in arm-base frame (m)
            target_z: vertical height (m)
            target_yaw: desired end-effector orientation (rad)
            elbow_up: True for elbow-up (+theta2), False for elbow-down (-theta2)
            
        Returns:
            (lift_z, theta1, theta2, theta3) or None if unreachable
        """
        # 1. Lift check
        if not (self.lift_min <= target_z <= self.lift_max):
            return None
        lift_z = target_z

        # 2. Planar reach check
        r2 = target_x**2 + target_y**2
        c2 = (r2 - self.l1**2 - self.l2**2) / (2.0 * self.l1 * self.l2)

        # Out of reach or too close
        if abs(c2) > 1.0:
            return None

        # 3. Elbow angle (theta2)
        # c2 can slightly exceed 1.0 due to float rounding
        c2 = max(-1.0, min(1.0, c2))
        s2 = math.sqrt(max(0.0, 1.0 - c2**2))
        theta2 = math.atan2(s2 if elbow_up else -s2, c2)

        # 4. Shoulder angle (theta1)
        theta1 = math.atan2(target_y, target_x) - math.atan2(self.l2 * math.sin(theta2),
                                                            self.l1 + self.l2 * math.cos(theta2))

        # 5. Wrist yaw (theta3) to maintain desired orientation
        theta3 = target_yaw - (theta1 + theta2)

        # Wrap angles to [-pi, pi]
        theta1 = math.atan2(math.sin(theta1), math.cos(theta1))
        theta2 = math.atan2(math.sin(theta2), math.cos(theta2))
        theta3 = math.atan2(math.sin(theta3), math.cos(theta3))

        # 6. Joint limits verification
        if not (self.theta1_limits[0] <= theta1 <= self.theta1_limits[1]):
            return None
        if not (self.theta2_limits[0] <= theta2 <= self.theta2_limits[1]):
            return None
        if not (self.wrist_limits[0] <= theta3 <= self.wrist_limits[1]):
            return None

        return lift_z, theta1, theta2, theta3

    def get_both_solutions(self, target_x: float, target_y: float, target_z: float,
                           target_yaw: float = 0.0) -> List[Tuple[float, float, float, float]]:
        """Return all valid IK solutions (elbow-up and elbow-down)."""
        solutions = []
        for elbow_up in [False, True]:
            sol = self.inverse_kinematics(target_x, target_y, target_z, target_yaw, elbow_up=elbow_up)
            if sol is not None:
                solutions.append(sol)
        return solutions

    def is_reachable(self, x: float, y: float, z: float) -> bool:
        """Quick check if target is within reach."""
        if not (self.lift_min <= z <= self.lift_max):
            return False
        dist = math.hypot(x, y)
        min_reach = abs(self.l1 - self.l2)
        max_reach = self.l1 + self.l2
        return min_reach <= dist <= max_reach
