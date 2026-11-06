"""
test_tracker.py — Unit tests for moving obstacle tracker and Kalman filtering.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import pytest
import numpy as np
from tracker import Tracker, CVKalmanTrack, cluster_points
from sensor_models import Track


def test_cluster_points():
    pts = np.array([
        [0.0, 0.0],
        [0.1, 0.1],
        [0.2, 0.0],
        [5.0, 5.0],
        [5.1, 5.0]
    ])
    clusters = cluster_points(pts, max_distance=0.5)
    assert len(clusters) == 2
    assert len(clusters[0]) == 3
    assert len(clusters[1]) == 2


def test_cv_kalman_track_predict_update():
    track = CVKalmanTrack(np.array([0.0, 0.0]), track_id=1, dt=0.1)
    track.predict()
    assert track.age == 1
    
    # Update with new detection
    innov = track.update(np.array([0.1, 0.0]))
    assert track.hits == 2
    pos = track.get_position()
    assert abs(pos[0] - 0.1) < 0.05


def test_tracker_velocity_recovery():
    cfg = {
        'tracker_cluster_distance': 0.5,
        'tracker_gate_threshold': 9.21,
        'tracker_max_age': 5,
        'tracker_min_hits': 2,
        'tracker_speed_threshold': 0.3,
        'timestep': 0.1
    }
    tracker = Tracker(cfg)
    
    # Move along x at 1.0 m/s (0.1m per 0.1s step)
    for i in range(8):
        dets = np.array([[1.0 + 0.1 * i, 2.0]])
        tracks = tracker.update(dets)
        
    assert len(tracks) == 1
    assert tracks[0].classification == 'moving'
    vx = tracks[0].velocity[0]
    assert 0.8 <= vx <= 1.2


def test_tracker_ttc():
    cfg = {
        'tracker_cluster_distance': 0.5,
        'tracker_gate_threshold': 9.21,
        'tracker_max_age': 5,
        'tracker_min_hits': 2,
        'tracker_speed_threshold': 0.3,
        'timestep': 0.1
    }
    tracker = Tracker(cfg)
    robot_pos = np.array([0.0, 0.0])
    robot_vel = np.array([0.0, 0.0])
    track = Track(
        track_id=1,
        position=np.array([4.0, 0.0]),
        velocity=np.array([-2.0, 0.0]),
        covariance=np.eye(4),
        timestamp=0.0,
        age=10,
        hits=10,
        classification='moving'
    )
    ttc = tracker.predict_ttc(robot_pos, robot_vel, track)
    assert ttc is not None
    assert 1.8 <= ttc <= 2.2
