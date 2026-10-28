"""
Obstacle fusion — multi-sensor per-sector obstacle detection with verification.

Per addendum §A6.6, fuses LiDAR, IR, sonar, and tracked obstacles using
inverse-variance weighting and verification logic:
- VERIFIED: ≥2 independent sensors agree
- UNVERIFIED: single sensor
- CONFLICT: strong disagreement
"""

import numpy as np
from typing import Dict, List, Tuple
from sensor_models import ObstacleReport


class ObstacleFusion:
    """
    Fuses obstacle detections from multiple sensors per sector.
    
    Sectors: front, front_left, front_right, left, right, rear_left, rear_right, rear
    """
    
    def __init__(self, config: dict):
        self.config = config
        
        # Sector definitions (angle ranges in radians, robot frame)
        self.sectors = {
            'front': (-np.pi/6, np.pi/6),
            'front_left': (np.pi/6, np.pi/2),
            'front_right': (-np.pi/2, -np.pi/6),
            'left': (np.pi/2, 2*np.pi/3),
            'right': (-2*np.pi/3, -np.pi/2),
            'rear_left': (2*np.pi/3, 5*np.pi/6),
            'rear_right': (-5*np.pi/6, -2*np.pi/3),
            'rear': (5*np.pi/6, -5*np.pi/6),  # Wraps around ±π
        }
        
        # Agreement threshold (meters)
        self.agree_threshold = config.get('obstacle_agree_threshold', 0.3)
        self.conflict_threshold = config.get('obstacle_conflict_threshold', 1.0)
        
    def fuse(self, sensor_readings: Dict[str, Tuple[float, float, float]]) -> Dict[str, ObstacleReport]:
        """
        Fuse obstacle detections across sensors.
        
        Args:
            sensor_readings: Dict mapping sensor_name -> (range, angle, trust)
                             where angle is in robot frame, trust is from health monitor
                             
        Returns:
            Dict mapping sector_name -> ObstacleReport
        """
        # Bin readings by sector
        sector_readings = {s: [] for s in self.sectors.keys()}
        
        for sensor_name, (r, angle, trust) in sensor_readings.items():
            if r <= 0 or not np.isfinite(r):
                continue
            sector = self._angle_to_sector(angle)
            if sector:
                # Store (range, variance, trust, sensor_name)
                # Assume variance ~ (0.1 * range)^2 as a default
                variance = (0.1 * r)**2
                sector_readings[sector].append((r, variance, trust, sensor_name))
                
        # Fuse per sector
        reports = {}
        for sector, readings in sector_readings.items():
            if len(readings) == 0:
                continue
                
            report = self._fuse_sector(sector, readings)
            reports[sector] = report
            
        return reports
        
    def _angle_to_sector(self, angle: float) -> str:
        """Map angle (rad, robot frame) to sector name."""
        # Normalize angle to [-π, π]
        angle = np.arctan2(np.sin(angle), np.cos(angle))
        
        for sector, (a_min, a_max) in self.sectors.items():
            if sector == 'rear':
                # Special case: wraps around
                if angle >= 5*np.pi/6 or angle <= -5*np.pi/6:
                    return 'rear'
            else:
                if a_min <= angle < a_max:
                    return sector
        return None
        
    def _fuse_sector(self, sector: str, readings: List[Tuple[float, float, float, str]]) -> ObstacleReport:
        """
        Fuse readings in a single sector using inverse-variance weighting.
        
        Args:
            readings: List of (range, variance, trust, sensor_name)
            
        Returns:
            ObstacleReport
        """
        if len(readings) == 1:
            # Single sensor
            r, var, trust, sensor = readings[0]
            return ObstacleReport(
                sector=sector,
                range=r,
                timestamp=0.0,  # Caller should set
                status='UNVERIFIED',
                trust=trust,
                sources=[sensor]
            )
            
        # Multiple sensors: check agreement
        ranges = np.array([r for r, _, _, _ in readings])
        trusts = np.array([t for _, _, t, _ in readings])
        sources = [s for _, _, _, s in readings]
        
        # Check for conflict
        if np.ptp(ranges) > self.conflict_threshold:
            # Strong disagreement
            # Use median and flag as conflict
            fused_range = np.median(ranges)
            return ObstacleReport(
                sector=sector,
                range=fused_range,
                timestamp=0.0,
                status='CONFLICT',
                trust=0.3,  # Low trust
                sources=sources
            )
            
        # Check for verification (agreement within threshold)
        if np.ptp(ranges) <= self.agree_threshold:
            status = 'VERIFIED'
        else:
            status = 'UNVERIFIED'
            
        # Inverse-variance fusion
        # weight_i = trust_i / variance_i
        weights = []
        for r, var, trust, _ in readings:
            if var > 0 and trust > 0:
                w = trust / var
            else:
                w = 0.0
            weights.append(w)
            
        weights = np.array(weights)
        if np.sum(weights) > 0:
            weights /= np.sum(weights)
            fused_range = np.sum(weights * ranges)
        else:
            fused_range = np.mean(ranges)
            
        # Aggregate trust: use max trust if verified, mean otherwise
        if status == 'VERIFIED':
            fused_trust = np.max(trusts)
        else:
            fused_trust = np.mean(trusts)
            
        return ObstacleReport(
            sector=sector,
            range=fused_range,
            timestamp=0.0,
            status=status,
            trust=fused_trust,
            sources=sources
        )


def test_obstacle_fusion():
    """Unit tests for obstacle fusion."""
    config = {
        'obstacle_agree_threshold': 0.3,
        'obstacle_conflict_threshold': 1.0
    }
    
    fusion = ObstacleFusion(config)
    
    # Test 1: Single sensor (UNVERIFIED)
    readings = {
        'lidar': (2.0, 0.0, 1.0)  # (range, angle=front, trust)
    }
    reports = fusion.fuse(readings)
    assert 'front' in reports
    assert reports['front'].status == 'UNVERIFIED'
    assert reports['front'].range == 2.0
    print(f"✓ Single sensor: {reports['front'].status}, range={reports['front'].range:.2f}")
    
    # Test 2: Two sensors agreeing (VERIFIED)
    readings = {
        'lidar': (2.0, 0.0, 1.0),
        'sonar': (2.1, 0.05, 0.9)
    }
    reports = fusion.fuse(readings)
    assert reports['front'].status == 'VERIFIED'
    assert 1.95 <= reports['front'].range <= 2.15
    print(f"✓ Verified: {reports['front'].status}, range={reports['front'].range:.2f}, sources={reports['front'].sources}")
    
    # Test 3: Two sensors disagreeing (CONFLICT)
    readings = {
        'lidar': (2.0, 0.0, 1.0),
        'sonar': (4.0, 0.0, 0.8)  # 2m difference
    }
    reports = fusion.fuse(readings)
    assert reports['front'].status == 'CONFLICT'
    assert reports['front'].trust < 0.5
    print(f"✓ Conflict: {reports['front'].status}, trust={reports['front'].trust:.2f}")
    
    # Test 4: Multiple sectors
    readings = {
        'lidar_front': (1.5, 0.0, 1.0),
        'lidar_left': (3.0, np.pi/2 + 0.1, 1.0),
        'ir_rear': (0.8, np.pi, 0.9)
    }
    reports = fusion.fuse(readings)
    assert 'front' in reports and 'left' in reports and 'rear' in reports
    print(f"✓ Multi-sector: {list(reports.keys())}")
    
    # Test 5: Sensor with degraded trust
    readings = {
        'lidar': (2.0, 0.0, 1.0),
        'sonar': (2.05, 0.0, 0.2)  # Low trust
    }
    reports = fusion.fuse(readings)
    # Should still verify but fused range biased toward high-trust sensor
    assert reports['front'].status == 'VERIFIED'
    assert reports['front'].range < 2.03  # Closer to lidar
    print(f"✓ Trust weighting: range={reports['front'].range:.3f} (should favor high-trust)")
    
    print("✓ All obstacle_fusion tests passed")


if __name__ == "__main__":
    test_obstacle_fusion()
