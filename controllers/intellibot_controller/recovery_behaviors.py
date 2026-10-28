"""
Recovery behaviors for stuck/blocked robot situations.

Per addendum §A11, implements recovery FSM:
- WAIT → ROTATE → BACKUP → REPLAN → ERROR
- Only backs up if rear is verified clear
- Configurable timeouts
- Integrates with sensor health and rear_safety
"""

import time
from enum import Enum, auto
from typing import Optional
from dataclasses import dataclass
import math


class RecoveryState(Enum):
    """Recovery FSM states."""
    IDLE = auto()
    WAIT = auto()
    ROTATE = auto()
    BACKUP = auto()
    REPLAN = auto()
    ERROR = auto()


@dataclass
class RecoveryConfig:
    """Recovery behavior configuration."""
    # Timeouts (seconds)
    wait_timeout: float = 3.0
    rotate_timeout: float = 5.0
    backup_timeout: float = 3.0
    replan_attempts: int = 3
    
    # Motion parameters
    rotate_speed: float = 0.3  # rad/s
    rotate_angle: float = math.pi / 4  # 45 degrees
    backup_speed: float = 0.1  # m/s (slow)
    backup_distance: float = 0.3  # m
    
    # Safety margins
    min_front_clearance: float = 0.2  # m, must have this to not be stuck
    min_rear_clearance: float = 0.3  # m, required for backup
    
    # Replanning
    cost_inflation_factor: float = 1.5  # inflate costs after failed attempt


class RecoveryBehavior:
    """
    Manages recovery from stuck/blocked situations.
    
    Usage:
        recovery = RecoveryBehavior(config)
        
        if path_blocked or stuck_detected:
            recovery.trigger()
        
        if recovery.is_active():
            cmd = recovery.update(dt, front_clear, rear_clear, current_pose)
            if cmd:
                v, omega = cmd
                # apply motion command
            
            if recovery.is_complete():
                if recovery.succeeded():
                    # continue with replanned path
                else:
                    # enter ERROR state
    """
    
    def __init__(self, config: Optional[RecoveryConfig] = None):
        self.config = config or RecoveryConfig()
        self.state = RecoveryState.IDLE
        self.state_start_time = 0.0
        self.replan_count = 0
        
        # State-specific tracking
        self.rotate_start_heading = 0.0
        self.rotate_direction = 1  # 1 = CCW, -1 = CW
        self.backup_start_pos = None
        
    def trigger(self):
        """Trigger recovery sequence (call when stuck/blocked)."""
        if self.state == RecoveryState.IDLE:
            self._enter_state(RecoveryState.WAIT)
            self.replan_count = 0
            
    def is_active(self) -> bool:
        """Check if recovery is currently running."""
        return self.state != RecoveryState.IDLE
        
    def is_complete(self) -> bool:
        """Check if recovery completed (success or error)."""
        return self.state in [RecoveryState.IDLE, RecoveryState.ERROR]
        
    def succeeded(self) -> bool:
        """Check if recovery succeeded (ready to continue)."""
        return self.state == RecoveryState.IDLE
        
    def reset(self):
        """Reset to IDLE (call after successful replan or manual intervention)."""
        self._enter_state(RecoveryState.IDLE)
        
    def get_state(self) -> RecoveryState:
        """Get current recovery state."""
        return self.state
        
    def update(self, dt: float, front_clearance: float, rear_clearance: float, 
               current_heading: float, current_pos: tuple) -> Optional[tuple]:
        """
        Update recovery behavior.
        
        Args:
            dt: Time step (s)
            front_clearance: Front obstacle distance (m), inf if clear
            rear_clearance: Rear obstacle distance (m), inf if clear
            current_heading: Robot heading (rad)
            current_pos: Robot position (x, y) in meters
            
        Returns:
            (v, omega) command if active, None if idle/error
        """
        if self.state == RecoveryState.IDLE:
            return None
            
        elapsed = time.time() - self.state_start_time
        
        # State machine
        if self.state == RecoveryState.WAIT:
            # Wait for dynamic obstacle to clear
            if front_clearance > self.config.min_front_clearance:
                # Obstacle cleared, try to continue
                self._enter_state(RecoveryState.IDLE)
                return None
            elif elapsed > self.config.wait_timeout:
                # Timeout, try rotating
                self._enter_state(RecoveryState.ROTATE)
                self.rotate_start_heading = current_heading
                # Alternate rotation direction on retries
                self.rotate_direction = 1 if (self.replan_count % 2 == 0) else -1
            return (0.0, 0.0)  # stop while waiting
            
        elif self.state == RecoveryState.ROTATE:
            # Rotate to find clearance
            angle_rotated = abs(self._angle_diff(current_heading, self.rotate_start_heading))
            
            if front_clearance > self.config.min_front_clearance:
                # Found clearance, request replan
                self._enter_state(RecoveryState.REPLAN)
                return None
            elif angle_rotated >= self.config.rotate_angle or elapsed > self.config.rotate_timeout:
                # Rotated enough or timeout, try backing up
                if rear_clearance > self.config.min_rear_clearance:
                    self._enter_state(RecoveryState.BACKUP)
                    self.backup_start_pos = current_pos
                else:
                    # Can't backup, go straight to replan with inflated costs
                    self._enter_state(RecoveryState.REPLAN)
                return None
            else:
                # Keep rotating
                omega = self.rotate_direction * self.config.rotate_speed
                return (0.0, omega)
                
        elif self.state == RecoveryState.BACKUP:
            # Back up to create space
            dx = current_pos[0] - self.backup_start_pos[0]
            dy = current_pos[1] - self.backup_start_pos[1]
            distance_backed = math.sqrt(dx*dx + dy*dy)
            
            if rear_clearance < self.config.min_rear_clearance:
                # Rear blocked during backup, abort to replan
                self._enter_state(RecoveryState.REPLAN)
                return None
            elif distance_backed >= self.config.backup_distance or elapsed > self.config.backup_timeout:
                # Backed up enough, request replan
                self._enter_state(RecoveryState.REPLAN)
                return None
            else:
                # Keep backing up (negative velocity)
                return (-self.config.backup_speed, 0.0)
                
        elif self.state == RecoveryState.REPLAN:
            # Request replan with inflated costs (handled by caller)
            # This is a momentary state; caller should call reset() after replanning
            self.replan_count += 1
            if self.replan_count >= self.config.replan_attempts:
                # Too many attempts, give up
                self._enter_state(RecoveryState.ERROR)
            else:
                # Return to WAIT to try the new path
                self._enter_state(RecoveryState.WAIT)
            return None
            
        elif self.state == RecoveryState.ERROR:
            # Terminal error state
            return None
            
        return None
        
    def _enter_state(self, new_state: RecoveryState):
        """Transition to a new state."""
        self.state = new_state
        self.state_start_time = time.time()
        
    def _angle_diff(self, a: float, b: float) -> float:
        """Compute signed angle difference a - b, wrapped to [-pi, pi]."""
        diff = a - b
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff


# ============================================================================
# UNIT TESTS
# ============================================================================

def test_wait_clears():
    """Test that WAIT→IDLE when obstacle clears."""
    config = RecoveryConfig(wait_timeout=5.0)
    recovery = RecoveryBehavior(config)
    
    recovery.trigger()
    assert recovery.get_state() == RecoveryState.WAIT
    
    # Obstacle present
    cmd = recovery.update(0.1, front_clearance=0.1, rear_clearance=1.0, 
                          current_heading=0.0, current_pos=(0, 0))
    assert cmd == (0.0, 0.0)
    assert recovery.is_active()
    
    # Obstacle clears
    cmd = recovery.update(0.1, front_clearance=0.5, rear_clearance=1.0,
                          current_heading=0.0, current_pos=(0, 0))
    assert recovery.get_state() == RecoveryState.IDLE
    assert recovery.succeeded()
    print("✓ WAIT clears when obstacle moves")


def test_wait_timeout_rotate():
    """Test that WAIT→ROTATE on timeout."""
    config = RecoveryConfig(wait_timeout=0.5)
    recovery = RecoveryBehavior(config)
    
    recovery.trigger()
    
    # Keep obstacle for timeout duration
    time.sleep(0.6)
    cmd = recovery.update(0.1, front_clearance=0.1, rear_clearance=1.0,
                          current_heading=0.0, current_pos=(0, 0))
    
    assert recovery.get_state() == RecoveryState.ROTATE
    print("✓ WAIT→ROTATE on timeout")


def test_rotate_finds_clearance():
    """Test that ROTATE→REPLAN when clearance found."""
    config = RecoveryConfig(wait_timeout=0.3)
    recovery = RecoveryBehavior(config)
    
    # Enter ROTATE directly
    recovery._enter_state(RecoveryState.ROTATE)
    recovery.rotate_start_heading = 0.0
    
    assert recovery.get_state() == RecoveryState.ROTATE
    
    # Simulate rotation, find clearance
    cmd = recovery.update(0.1, front_clearance=0.5, rear_clearance=1.0,
                          current_heading=0.3, current_pos=(0, 0))
    
    assert recovery.get_state() == RecoveryState.REPLAN
    print("✓ ROTATE→REPLAN when clearance found")


def test_rotate_timeout_backup():
    """Test that ROTATE→BACKUP on timeout if rear clear."""
    config = RecoveryConfig(rotate_timeout=0.5, rotate_angle=math.pi)
    recovery = RecoveryBehavior(config)
    
    recovery._enter_state(RecoveryState.ROTATE)
    recovery.rotate_start_heading = 0.0
    
    time.sleep(0.6)
    cmd = recovery.update(0.1, front_clearance=0.1, rear_clearance=1.0,
                          current_heading=0.2, current_pos=(0, 0))
    
    assert recovery.get_state() == RecoveryState.BACKUP
    print("✓ ROTATE→BACKUP on timeout with rear clear")


def test_backup_rear_blocked():
    """Test that BACKUP→REPLAN if rear becomes blocked."""
    config = RecoveryConfig()
    recovery = RecoveryBehavior(config)
    
    recovery._enter_state(RecoveryState.BACKUP)
    recovery.backup_start_pos = (0.0, 0.0)
    
    # Rear becomes blocked
    cmd = recovery.update(0.1, front_clearance=0.1, rear_clearance=0.1,
                          current_heading=0.0, current_pos=(0, 0))
    
    assert recovery.get_state() == RecoveryState.REPLAN
    print("✓ BACKUP→REPLAN when rear blocked")


def test_backup_completes():
    """Test that BACKUP→REPLAN after distance reached."""
    config = RecoveryConfig(backup_distance=0.3)
    recovery = RecoveryBehavior(config)
    
    recovery._enter_state(RecoveryState.BACKUP)
    recovery.backup_start_pos = (0.0, 0.0)
    
    # Simulate backing up
    cmd = recovery.update(0.1, 0.1, 1.0, 0.0, (-0.1, 0.0))
    assert cmd == (-config.backup_speed, 0.0)
    
    cmd = recovery.update(0.1, 0.1, 1.0, 0.0, (-0.35, 0.0))
    assert recovery.get_state() == RecoveryState.REPLAN
    print("✓ BACKUP→REPLAN after distance")


def test_replan_max_attempts():
    """Test that REPLAN→ERROR after max attempts."""
    config = RecoveryConfig(replan_attempts=2)
    recovery = RecoveryBehavior(config)
    
    recovery._enter_state(RecoveryState.REPLAN)
    
    # First attempt
    recovery.update(0.1, 0.1, 1.0, 0.0, (0, 0))
    assert recovery.get_state() == RecoveryState.WAIT
    assert recovery.replan_count == 1
    
    # Force back to replan
    recovery._enter_state(RecoveryState.REPLAN)
    recovery.update(0.1, 0.1, 1.0, 0.0, (0, 0))
    assert recovery.get_state() == RecoveryState.ERROR
    assert not recovery.succeeded()
    print("✓ REPLAN→ERROR after max attempts")


def test_recovery_full_sequence():
    """Test full recovery sequence WAIT→ROTATE→BACKUP→REPLAN."""
    config = RecoveryConfig(
        wait_timeout=0.3,
        rotate_timeout=0.3,
        backup_distance=0.2
    )
    recovery = RecoveryBehavior(config)
    
    # Trigger
    recovery.trigger()
    assert recovery.get_state() == RecoveryState.WAIT
    
    # WAIT timeout
    time.sleep(0.4)
    recovery.update(0.1, 0.1, 1.0, 0.0, (0, 0))
    assert recovery.get_state() == RecoveryState.ROTATE
    
    # ROTATE timeout
    time.sleep(0.4)
    recovery.update(0.1, 0.1, 1.0, 0.3, (0, 0))
    assert recovery.get_state() == RecoveryState.BACKUP
    
    # BACKUP completes
    recovery.update(0.1, 0.1, 1.0, 0.0, (-0.25, 0.0))
    assert recovery.get_state() == RecoveryState.REPLAN
    
    print("✓ Full sequence WAIT→ROTATE→BACKUP→REPLAN")


if __name__ == "__main__":
    print("Testing recovery_behaviors.py...")
    test_wait_clears()
    test_wait_timeout_rotate()
    test_rotate_finds_clearance()
    test_rotate_timeout_backup()
    test_backup_rear_blocked()
    test_backup_completes()
    test_replan_max_attempts()
    test_recovery_full_sequence()
    print("\n✅ All recovery behavior tests passed!")
