"""
test_costmap.py — Unit tests for layered Costmap.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import pytest
import numpy as np
from mapping import OccupancyGrid
from costmap import Costmap, StaticLayer, InflationLayer, DynamicLayer, PredictedDynamicLayer
from sensor_models import Track


def test_static_layer():
    grid = OccupancyGrid(x_min=0.0, x_max=10.0, y_min=0.0, y_max=10.0, res=0.1)
    grid.add_obstacle_world(5.0, 5.0, radius=0.5)
    
    costmap = Costmap(grid, inflation_radius=0.5)
    row, col = grid.world_to_cell(5.0, 5.0)
    assert costmap.static_layer.get_cost(col, row) == 255
    assert costmap.is_lethal(col, row)


def test_inflation_layer():
    grid = OccupancyGrid(x_min=0.0, x_max=10.0, y_min=0.0, y_max=10.0, res=0.1)
    grid.add_obstacle_world(5.0, 5.0, radius=0.2)
    
    costmap = Costmap(grid, inflation_radius=0.5)
    row, col = grid.world_to_cell(5.0, 5.0)
    
    # Adjacent cells should have cost decaying with distance
    cost_adjacent = costmap.inflation_layer.get_cost(col + 1, row)
    assert 0 < cost_adjacent < 255
    
    # Far cell should be 0
    far_row, far_col = grid.world_to_cell(1.0, 1.0)
    assert costmap.inflation_layer.get_cost(far_col, far_row) == 0


def test_dynamic_layer_add_and_clear():
    grid = OccupancyGrid(x_min=0.0, x_max=10.0, y_min=0.0, y_max=10.0, res=0.1)
    costmap = Costmap(grid, inflation_radius=0.3)
    
    costmap.update_dynamic([(7.0, 7.0)], radius=0.3)
    row_dyn, col_dyn = grid.world_to_cell(7.0, 7.0)
    assert costmap.dynamic_layer.get_cost(col_dyn, row_dyn) == 255
    assert costmap.get_cost(col_dyn, row_dyn) == 255
    
    # Clear dynamic
    costmap.clear_dynamic()
    assert costmap.dynamic_layer.get_cost(col_dyn, row_dyn) == 0


def test_predicted_dynamic_layer():
    grid = OccupancyGrid(x_min=0.0, x_max=10.0, y_min=0.0, y_max=10.0, res=0.1)
    costmap = Costmap(grid, inflation_radius=0.3)
    
    track = Track(
        track_id=1,
        position=np.array([4.0, 4.0]),
        velocity=np.array([1.0, 0.0]),
        covariance=np.eye(4),
        timestamp=0.0,
        age=5,
        hits=5,
        classification='moving'
    )
    
    costmap.update_predicted([track])
    # Future position at t=1s should be (5.0, 4.0)
    row_pred, col_pred = grid.world_to_cell(5.0, 4.0)
    assert costmap.predicted_layer.get_cost(col_pred, row_pred) > 0
