"""
test_rear_safety.py — Unit tests for rear safety monitor and TTC.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import pytest
import numpy as np
from rear_safety import RearSafetyMonitor, compute_rear_approach_ttc
from sensor_models import ObstacleReport


@pytest.fixture
def monitor():
    config = {
        'REAR_CLEAR_DIST': 1.0,
        'REAR_WARN_DIST': 2.0,
        'REAR_CRITICAL_TTC': 3.0,
        'REAR_WARN_TTC': 5.0
    }
    return RearSafetyMonitor(config)


def test_clear_rear(monitor):
    rear_obstacles = {
        'rear': ObstacleReport('rear', 3.0, 0.0, 'VERIFIED', 1.0, ['lidar', 'sonar'])
    }
    rear_clear = monitor.update(rear_obstacles, (0.0, 0.0), [])
    assert rear_clear
    assert monitor.warning_level == 'OK'


def test_obstacle_close(monitor):
    rear_obstacles = {
        'rear': ObstacleReport('rear', 0.5, 0.0, 'VERIFIED', 1.0, ['lidar'])
    }
    rear_clear = monitor.update(rear_obstacles, (0.0, 0.0), [])
    assert not rear_clear
    assert monitor.warning_level in ['WARN', 'CRITICAL']


def test_ttc_rear_approach():
    robot_pos = np.array([0.0, 0.0])
    robot_vel = np.array([0.5, 0.0])
    robot_heading = 0.0
    
    obstacle_pos = np.array([-2.0, 0.0])
    obstacle_vel = np.array([1.5, 0.0])
    
    ttc = compute_rear_approach_ttc(robot_pos, robot_vel, obstacle_pos, obstacle_vel, robot_heading)
    assert ttc is not None
    assert 1.8 <= ttc <= 2.2
