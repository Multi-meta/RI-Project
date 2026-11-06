"""
test_sensor_health.py — Unit tests for SensorHealthMonitor.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import pytest
from sensor_health import SensorHealthMonitor, CameraQualityMonitor
from sensor_models import SensorReading


def test_sensor_health_transitions():
    cfg = {
        'stale_threshold': 1.0,
        'plausibility_range': (0.0, 5.0),
        'max_rate': 2.0,
        'nis_threshold': 7.0
    }
    mon = SensorHealthMonitor('lidar', cfg)
    
    # Nominal
    r = SensorReading(timestamp=1.0, valid=True, quality=1.0)
    h = mon.update(r, 2.5)
    assert h.status == 'OK'
    assert h.trust == 1.0
    assert mon.get_r_inflation() == 1.0
    
    # Out of range -> DEGRADED
    r2 = SensorReading(timestamp=2.0, valid=True, quality=1.0)
    h2 = mon.update(r2, 12.0)
    assert h2.status == 'DEGRADED'
    
    # Invalid -> FAILED
    r3 = SensorReading(timestamp=3.0, valid=False, quality=0.0)
    h3 = mon.update(r3, None)
    assert h3.status == 'FAILED'
    assert mon.get_r_inflation() >= 1e6
