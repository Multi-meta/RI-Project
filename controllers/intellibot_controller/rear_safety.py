"""
Rear safety — rear-approach detection and response.

Per addendum §A8.5, combines rear camera, rear IR/sonar, and rear LiDAR
to detect approaching obstacles from behind. Computes closing speed and TTC.

Rules:
- No reverse unless rear is clear
- Warn/yield on TTC < threshold
- LED/log/broadcast warnings
"""

import numpy as np
from typing import Optional, Tuple, Dict
from sensor_models import ObstacleReport


class RearSafetyMonitor:
    """
    Monitors rear approaches and enforces safety rules.
    
    Inputs: rear sensor readings, tracked obstacles, robot velocity
    Outputs: rear_clear flag, warnings, TTC estimates
    """
    
    def __init__(self, config: dict):
        self.config = config
        
        # Thresholds
        self.clear_distance = config.get('REAR_CLEAR_DIST', 1.0)  # meters
        self.warn_distance = config.get('REAR_WARN_DIST', 2.0)
        self.critical_ttc = config.get('REAR_CRITICAL_TTC', 3.0)  # seconds
        self.warn_ttc = config.get('REAR_WARN_TTC', 5.0)
        
        # State
        self.rear_clear = True
        self.rear_distance = np.inf
        self.closing_speed = 0.0
        self.ttc = None
        self.warning_level = 'OK'  # 'OK', 'WARN', 'CRITICAL'
        
    def update(self, rear_obstacles: Dict[str, ObstacleReport],
               robot_velocity: Tuple[float, float],
               tracked_obstacles: list) -> bool:
        """
        Update rear safety status.
        
        Args:
            rear_obstacles: Dict of ObstacleReport from rear-facing sensors
            robot_velocity: (vx, vy) robot velocity in world frame
            tracked_obstacles: List of Track objects
            
        Returns:
            rear_clear: bool, whether reverse is allowed
        """
        # Find minimum distance from rear sectors
        rear_sectors = ['rear', 'rear_left', 'rear_right']
        min_distance = np.inf
        
        for sector, report in rear_obstacles.items():
            if sector in rear_sectors and report.status in ['VERIFIED', 'UNVERIFIED']:
                if report.range < min_distance:
                    min_distance = report.range
                    
        self.rear_distance = min_distance
        
        # Check for approaching tracked obstacles from behind
        # (Requires world-frame tracks and robot pose/velocity)
        # For now, simplified: use sensor range only
        
        # Estimate closing speed (heuristic: assume obstacle stationary or moving)
        # If we have previous distance, compute rate of change
        # Simplified: if distance is decreasing quickly, estimate from that
        
        # Determine rear_clear flag
        if self.rear_distance < self.clear_distance:
            self.rear_clear = False
        else:
            self.rear_clear = True
            
        # Compute TTC if approaching
        if self.rear_distance < self.warn_distance:
            # Simplified TTC: assume obstacle is closing at a conservative speed
            # (In full implementation, use tracked obstacle velocity)
            assumed_closing_speed = 1.0  # m/s conservative estimate
            self.ttc = self.rear_distance / assumed_closing_speed
        else:
            self.ttc = None
            
        # Determine warning level
        if self.ttc is not None:
            if self.ttc < self.critical_ttc:
                self.warning_level = 'CRITICAL'
            elif self.ttc < self.warn_ttc:
                self.warning_level = 'WARN'
            else:
                self.warning_level = 'OK'
        else:
            if not self.rear_clear:
                self.warning_level = 'WARN'
            else:
                self.warning_level = 'OK'
                
        return self.rear_clear
        
    def get_status(self) -> dict:
        """Return rear safety status for logging/broadcast."""
        return {
            'rear_clear': self.rear_clear,
            'rear_distance': self.rear_distance,
            'ttc': self.ttc,
            'warning_level': self.warning_level
        }
        
    def should_yield(self) -> bool:
        """Return whether robot should yield/stop due to rear approach."""
        return self.warning_level == 'CRITICAL'
        
    def should_warn(self) -> bool:
        """Return whether to issue a warning (LED/speaker)."""
        return self.warning_level in ['WARN', 'CRITICAL']


def compute_rear_approach_ttc(robot_pos: np.ndarray, robot_vel: np.ndarray,
                               obstacle_pos: np.ndarray, obstacle_vel: np.ndarray,
                               robot_heading: float) -> Optional[float]:
    """
    Compute TTC for a rear-approaching obstacle.
    
    Args:
        robot_pos: [x, y] robot position
        robot_vel: [vx, vy] robot velocity
        obstacle_pos: [x, y] obstacle position
        obstacle_vel: [vx, vy] obstacle velocity
        robot_heading: robot heading (rad)
        
    Returns:
        TTC in seconds, or None if not approaching from rear
    """
    # Vector from robot to obstacle
    rel_pos = obstacle_pos - robot_pos
    
    # Check if obstacle is behind robot
    # Robot's rear direction is opposite to heading
    rear_dir = np.array([-np.cos(robot_heading), -np.sin(robot_heading)])
    
    # Dot product: positive if obstacle is in rear hemisphere
    behind = np.dot(rel_pos, rear_dir) > 0
    
    if not behind:
        return None
        
    # Relative velocity
    rel_vel = obstacle_vel - robot_vel
    
    # Closing speed: component of relative velocity along relative position
    distance = np.linalg.norm(rel_pos)
    closing_speed = -np.dot(rel_vel, rel_pos) / distance
    
    if closing_speed <= 0:
        return None  # Not closing
        
    ttc = distance / closing_speed
    return ttc


def test_rear_safety():
    """Unit tests for rear safety."""
    print("Testing rear safety...")
    
    config = {
        'REAR_CLEAR_DIST': 1.0,
        'REAR_WARN_DIST': 2.0,
        'REAR_CRITICAL_TTC': 3.0,
        'REAR_WARN_TTC': 5.0
    }
    
    monitor = RearSafetyMonitor(config)
    
    # Test 1: Clear rear
    from sensor_models import ObstacleReport
    rear_obstacles = {
        'rear': ObstacleReport('rear', 3.0, 0.0, 'VERIFIED', 1.0, ['lidar', 'sonar'])
    }
    rear_clear = monitor.update(rear_obstacles, (0.0, 0.0), [])
    assert rear_clear
    assert monitor.warning_level == 'OK'
    print(f"✓ Clear rear: {monitor.get_status()}")
    
    # Test 2: Obstacle close behind
    rear_obstacles = {
        'rear': ObstacleReport('rear', 0.5, 0.0, 'VERIFIED', 1.0, ['lidar'])
    }
    rear_clear = monitor.update(rear_obstacles, (0.0, 0.0), [])
    assert not rear_clear
    assert monitor.warning_level in ['WARN', 'CRITICAL']
    print(f"✓ Obstacle close: {monitor.get_status()}")
    
    # Test 3: Approaching (medium distance)
    rear_obstacles = {
        'rear': ObstacleReport('rear', 1.5, 0.0, 'VERIFIED', 1.0, ['lidar'])
    }
    rear_clear = monitor.update(rear_obstacles, (0.0, 0.0), [])
    assert monitor.ttc is not None
    assert monitor.should_warn()
    print(f"✓ Approaching: TTC={monitor.ttc:.1f}s, warn={monitor.should_warn()}, yield={monitor.should_yield()}")
    
    # Test 4: TTC computation
    robot_pos = np.array([0.0, 0.0])
    robot_vel = np.array([0.5, 0.0])  # Moving forward
    robot_heading = 0.0  # Facing +x
    
    # Obstacle behind, approaching faster
    obstacle_pos = np.array([-2.0, 0.0])
    obstacle_vel = np.array([1.5, 0.0])  # Moving forward faster
    
    ttc = compute_rear_approach_ttc(robot_pos, robot_vel, obstacle_pos, obstacle_vel, robot_heading)
    assert ttc is not None and 1.5 <= ttc <= 2.5  # Should be ~2s
    print(f"✓ TTC computation: {ttc:.2f}s")
    
    # Test 5: Obstacle not approaching from rear (ahead)
    obstacle_pos = np.array([2.0, 0.0])  # Ahead
    ttc = compute_rear_approach_ttc(robot_pos, robot_vel, obstacle_pos, obstacle_vel, robot_heading)
    assert ttc is None
    print("✓ Obstacle ahead: TTC=None")
    
    # Test 6: Obstacle behind but moving away
    obstacle_pos = np.array([-2.0, 0.0])
    obstacle_vel = np.array([0.2, 0.0])  # Moving slower than robot
    ttc = compute_rear_approach_ttc(robot_pos, robot_vel, obstacle_pos, obstacle_vel, robot_heading)
    assert ttc is None  # Moving apart
    print("✓ Moving apart: TTC=None")
    
    print("✓ All rear_safety tests passed")


if __name__ == "__main__":
    test_rear_safety()
