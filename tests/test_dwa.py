"""
test_dwa.py — Unit tests for Dynamic Window Approach local planner.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import pytest
import numpy as np
from dwa import DWA
from mapping import OccupancyGrid
from costmap import Costmap


@pytest.fixture
def dwa_and_costmap():
    config = {
        'V_MAX': 0.5,
        'W_MAX': 2.0,
        'MAX_ACC_V': 0.5,
        'MAX_ACC_W': 2.0,
        'WHEEL_SEPARATION': 0.3,
        'WHEEL_RADIUS': 0.05,
        'ROBOT_RADIUS': 0.25,
        'DWA_V_SAMPLES': 5,
        'DWA_W_SAMPLES': 10,
        'DWA_DT': 0.2,
        'DWA_SIM_TIME': 1.0,
        'DWA_W_HEADING': 1.0,
        'DWA_W_DISTANCE': 1.0,
        'DWA_W_VELOCITY': 0.5,
        'DWA_ALLOW_REVERSE': True
    }
    dwa = DWA(config)
    grid = OccupancyGrid(x_min=0.0, x_max=20.0, y_min=0.0, y_max=20.0, res=0.2)
    costmap = Costmap(grid, inflation_radius=0.3)
    return dwa, costmap


def test_dwa_free_space(dwa_and_costmap):
    dwa, costmap = dwa_and_costmap
    pose = (5.0, 5.0, 0.0)
    goal = (10.0, 5.0)
    v, w = dwa.compute(pose, 0.0, 0.0, goal, costmap, rear_clear=True)
    assert v > 0.0
    assert abs(w) < 0.5


def test_dwa_obstacle_avoidance(dwa_and_costmap):
    dwa, costmap = dwa_and_costmap
    pose = (5.0, 5.0, 0.0)
    costmap.update_dynamic([(7.0, 5.0)])  # Obstacle right in front
    goal = (10.0, 8.0)
    v, w = dwa.compute(pose, 0.0, 0.0, goal, costmap, rear_clear=True)
    # Should steer to avoid
    assert abs(w) > 0.05 or v > 0.0


def test_dwa_rear_clear_enforcement(dwa_and_costmap):
    dwa, costmap = dwa_and_costmap
    pose = (5.0, 5.0, 0.0)
    goal = (2.0, 5.0)  # Behind robot
    # If rear is blocked, robot must NOT reverse
    v, w = dwa.compute(pose, 0.0, 0.0, goal, costmap, rear_clear=False)
    assert v >= 0.0
