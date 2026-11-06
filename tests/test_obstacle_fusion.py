"""
test_obstacle_fusion.py — Unit tests for multi-sensor obstacle fusion.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import pytest
import numpy as np
from obstacle_fusion import ObstacleFusion


def test_fusion_verified_and_conflict():
    cfg = {
        'agree_threshold': 0.3,
        'conflict_threshold': 1.0,
        'sensor_weights': {'lidar': 1.0, 'sonar': 0.8, 'ir': 0.6}
    }
    fusion = ObstacleFusion(cfg)
    
    # 2 agreeing sensors
    readings = {
        'lidar': (2.0, 0.0, 1.0),
        'sonar': (2.1, 0.05, 0.9)
    }
    rep = fusion.fuse(readings)
    assert 'front' in rep
    assert rep['front'].status == 'VERIFIED'
    
    # 2 conflicting sensors
    readings_conf = {
        'lidar': (2.0, 0.0, 1.0),
        'sonar': (4.0, 0.0, 0.8)
    }
    rep_conf = fusion.fuse(readings_conf)
    assert rep_conf['front'].status == 'CONFLICT'
