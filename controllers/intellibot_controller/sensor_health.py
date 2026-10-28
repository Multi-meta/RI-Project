"""
Sensor health monitoring — staleness, validity, plausibility, NIS tracking.

Per addendum §A6.4, each sensor has health status (OK/DEGRADED/FAILED)
and a trust value (0-1) that affects measurement covariance (R inflation).
"""

import time
import numpy as np
from dataclasses import dataclass
from collections import deque


@dataclass
class HealthStatus:
    """Sensor health status."""
    sensor_name: str
    status: str  # 'OK', 'DEGRADED', 'FAILED'
    trust: float  # 0-1
    reason: str
    timestamp: float


class SensorHealthMonitor:
    """
    Monitors sensor health across multiple dimensions:
    - Staleness: time since last valid reading
    - Validity: valid flag from sensor
    - Plausibility: range checks, rate limits
    - NIS (Normalized Innovation Squared): consistency with filter
    """
    
    def __init__(self, sensor_name: str, config: dict):
        self.sensor_name = sensor_name
        self.config = config
        
        # Thresholds from config
        self.stale_threshold = config.get('stale_threshold', 1.0)  # seconds
        self.plausibility_range = config.get('plausibility_range', (0.0, 10.0))
        self.max_rate = config.get('max_rate', 5.0)  # units per second
        self.nis_window = config.get('nis_window', 20)
        self.nis_threshold = config.get('nis_threshold', 7.815)  # χ² for 3 DOF at 95%
        
        # State
        self.last_reading_time = None
        self.last_value = None
        self.status = 'OK'
        self.trust = 1.0
        self.nis_history = deque(maxlen=self.nis_window)
        
    def update(self, reading, value, nis=None):
        """
        Update health based on a new sensor reading.
        
        Args:
            reading: SensorReading with timestamp, valid, quality
            value: The actual sensor value (for plausibility checks)
            nis: Normalized Innovation Squared from Kalman filter (optional)
            
        Returns:
            HealthStatus
        """
        current_time = reading.timestamp
        issues = []
        
        # Check staleness
        if self.last_reading_time is not None:
            dt = current_time - self.last_reading_time
            if dt > self.stale_threshold:
                issues.append(f"stale_{dt:.1f}s")
                
        # Check validity
        if not reading.valid:
            issues.append("invalid")
            
        # Check quality (sensor-specific)
        if reading.quality is not None and reading.quality < 0.3:
            issues.append(f"low_quality_{reading.quality:.2f}")
            
        # Plausibility checks
        if value is not None and reading.valid:
            # Range check
            if isinstance(value, (int, float)):
                if not (self.plausibility_range[0] <= value <= self.plausibility_range[1]):
                    issues.append(f"out_of_range_{value:.2f}")
                    
            # Rate check
            if self.last_value is not None and self.last_reading_time is not None:
                dt = current_time - self.last_reading_time
                if dt > 0:
                    if isinstance(value, (int, float)) and isinstance(self.last_value, (int, float)):
                        rate = abs(value - self.last_value) / dt
                        if rate > self.max_rate:
                            issues.append(f"high_rate_{rate:.2f}")
                            
        # NIS check (innovation consistency)
        if nis is not None:
            self.nis_history.append(nis)
            if len(self.nis_history) >= 5:  # Need some history
                mean_nis = np.mean(self.nis_history)
                if mean_nis > self.nis_threshold:
                    issues.append(f"high_NIS_{mean_nis:.2f}")
                    
        # Determine status and trust
        if 'invalid' in str(issues) or 'stale' in str(issues):
            self.status = 'FAILED'
            self.trust = 0.0
        elif len(issues) > 0:
            self.status = 'DEGRADED'
            # Degrade trust proportionally
            self.trust = max(0.1, 1.0 - 0.2 * len(issues))
        else:
            self.status = 'OK'
            self.trust = 1.0
            
        # Update history
        self.last_reading_time = current_time
        if value is not None and reading.valid:
            self.last_value = value
            
        reason = '; '.join(issues) if issues else 'nominal'
        
        return HealthStatus(
            sensor_name=self.sensor_name,
            status=self.status,
            trust=self.trust,
            reason=reason,
            timestamp=current_time
        )
        
    def get_r_inflation(self) -> float:
        """
        Return R-matrix inflation factor based on health.
        
        Lower trust → higher measurement covariance → less Kalman weight.
        """
        if self.status == 'FAILED':
            return 1e6  # Effectively ignore
        elif self.status == 'DEGRADED':
            # Inflate by inverse of trust
            return 1.0 / max(0.01, self.trust)
        else:
            return 1.0
            
    def reset(self):
        """Reset health monitor."""
        self.last_reading_time = None
        self.last_value = None
        self.status = 'OK'
        self.trust = 1.0
        self.nis_history.clear()


class CameraQualityMonitor:
    """
    Specialized health monitor for camera based on brightness/contrast.
    
    Used to detect lights-off scenarios and adjust detector/landmark trust.
    """
    
    def __init__(self):
        self.brightness_threshold = (0.1, 0.95)  # too dark or too bright
        self.contrast_threshold = 0.05  # too low
        
    def assess(self, brightness: float, contrast: float) -> HealthStatus:
        """Assess camera quality from brightness/contrast metrics."""
        issues = []
        trust = 1.0
        
        if brightness < self.brightness_threshold[0]:
            issues.append(f"too_dark_{brightness:.2f}")
            trust *= (brightness / self.brightness_threshold[0])
        elif brightness > self.brightness_threshold[1]:
            issues.append(f"too_bright_{brightness:.2f}")
            trust *= 0.5
            
        if contrast < self.contrast_threshold:
            issues.append(f"low_contrast_{contrast:.2f}")
            trust *= (contrast / self.contrast_threshold)
            
        if issues:
            status = 'DEGRADED'
        else:
            status = 'OK'
            
        trust = max(0.0, min(1.0, trust))
        reason = '; '.join(issues) if issues else 'nominal'
        
        return HealthStatus(
            sensor_name='camera',
            status=status,
            trust=trust,
            reason=reason,
            timestamp=time.time()
        )


def test_sensor_health():
    """Unit tests for sensor health monitoring."""
    from sensor_models import SensorReading
    
    config = {
        'stale_threshold': 1.0,
        'plausibility_range': (0.0, 5.0),
        'max_rate': 2.0,
        'nis_threshold': 7.0
    }
    
    monitor = SensorHealthMonitor('test_sensor', config)
    
    # Test 1: Nominal reading
    reading = SensorReading(timestamp=1.0, valid=True, quality=1.0)
    health = monitor.update(reading, 2.5)
    assert health.status == 'OK' and health.trust == 1.0
    print(f"✓ Nominal: {health.status}, trust={health.trust:.2f}")
    
    # Test 2: Out of range
    reading = SensorReading(timestamp=2.0, valid=True, quality=1.0)
    health = monitor.update(reading, 10.0)  # out of range
    assert health.status == 'DEGRADED'
    print(f"✓ Out of range: {health.status}, {health.reason}")
    
    # Test 3: Invalid
    reading = SensorReading(timestamp=3.0, valid=False, quality=0.0)
    health = monitor.update(reading, None)
    assert health.status == 'FAILED' and health.trust == 0.0
    print(f"✓ Invalid: {health.status}, trust={health.trust:.2f}")
    
    # Test 4: R inflation
    monitor.status = 'OK'
    monitor.trust = 1.0
    assert monitor.get_r_inflation() == 1.0
    
    monitor.status = 'DEGRADED'
    monitor.trust = 0.5
    assert monitor.get_r_inflation() == 2.0
    
    monitor.status = 'FAILED'
    assert monitor.get_r_inflation() >= 1e6
    print("✓ R inflation scales correctly")
    
    # Test 5: Camera quality
    cam_monitor = CameraQualityMonitor()
    health = cam_monitor.assess(brightness=0.05, contrast=0.15)
    assert health.status == 'DEGRADED' and 'too_dark' in health.reason
    print(f"✓ Camera dark: {health.status}, trust={health.trust:.2f}, {health.reason}")
    
    health = cam_monitor.assess(brightness=0.5, contrast=0.2)
    assert health.status == 'OK'
    print(f"✓ Camera OK: {health.status}, trust={health.trust:.2f}")
    
    print("✓ All sensor_health tests passed")


if __name__ == "__main__":
    test_sensor_health()
