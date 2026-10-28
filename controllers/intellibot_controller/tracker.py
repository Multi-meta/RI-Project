"""
Tracker — moving obstacle detection and tracking with constant-velocity Kalman filter.

Per addendum §A8.2, implements:
- Clustering of LiDAR/sensor detections
- Data association (nearest-neighbor with gating)
- Constant-velocity Kalman tracks
- Static/moving classification
- TTC (Time To Collision) prediction
"""

import numpy as np
from typing import List, Tuple, Optional
from sensor_models import Track
from kinematics import wrap_to_pi


def cluster_points(points: np.ndarray, max_distance: float = 0.5) -> List[np.ndarray]:
    """
    Simple clustering: group points within max_distance.
    
    Args:
        points: Nx2 array of (x, y) points
        max_distance: Maximum distance between points in a cluster
        
    Returns:
        List of clusters (each a Mx2 array)
    """
    if len(points) == 0:
        return []
        
    clusters = []
    remaining = list(range(len(points)))
    
    while remaining:
        # Start new cluster with first remaining point
        seed = remaining.pop(0)
        cluster = [seed]
        
        # Find all points within max_distance
        i = 0
        while i < len(remaining):
            idx = remaining[i]
            # Check distance to any point in current cluster
            min_dist = min(np.linalg.norm(points[idx] - points[c]) for c in cluster)
            if min_dist <= max_distance:
                cluster.append(idx)
                remaining.pop(i)
            else:
                i += 1
                
        clusters.append(points[cluster])
        
    return clusters


class CVKalmanTrack:
    """
    Constant-velocity Kalman filter for tracking a moving obstacle.
    
    State: [x, y, vx, vy]
    Measurement: [x, y]
    """
    
    def __init__(self, initial_pos: np.ndarray, track_id: int, dt: float = 0.1):
        """
        Args:
            initial_pos: [x, y]
            track_id: Unique track ID
            dt: Time step (s)
        """
        self.track_id = track_id
        self.dt = dt
        
        # State: [x, y, vx, vy]
        self.x = np.array([initial_pos[0], initial_pos[1], 0.0, 0.0])
        
        # Covariance
        self.P = np.diag([0.5**2, 0.5**2, 1.0**2, 1.0**2])
        
        # Process noise
        q_pos = 0.1**2
        q_vel = 0.5**2
        self.Q = np.diag([q_pos, q_pos, q_vel, q_vel])
        
        # Measurement noise
        self.R = np.diag([0.2**2, 0.2**2])
        
        # State transition matrix
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Measurement matrix
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Metadata
        self.age = 0
        self.hits = 1  # Starts with 1 (initial detection)
        self.time_since_update = 0
        self.classification = 'unknown'
        
    def predict(self):
        """Prediction step."""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.age += 1
        self.time_since_update += 1
        
    def update(self, z: np.ndarray):
        """
        Update step with measurement z = [x, y].
        
        Returns:
            innovation: np.ndarray
        """
        # Innovation
        y = z - self.H @ self.x
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.solve(S, np.eye(2))
        
        # Update state
        self.x = self.x + K @ y
        
        # Update covariance
        I_KH = np.eye(4) - K @ self.H
        self.P = I_KH @ self.P
        
        # Update metadata
        self.hits += 1
        self.time_since_update = 0
        
        return y
        
    def get_position(self) -> np.ndarray:
        """Return current position estimate."""
        return self.x[:2]
        
    def get_velocity(self) -> np.ndarray:
        """Return current velocity estimate."""
        return self.x[2:]
        
    def get_speed(self) -> float:
        """Return speed (magnitude of velocity)."""
        return np.linalg.norm(self.get_velocity())
        
    def classify(self, speed_threshold: float = 0.3):
        """
        Classify as static or moving based on speed.
        
        Args:
            speed_threshold: Minimum speed (m/s) to be considered moving
        """
        speed = self.get_speed()
        if speed < speed_threshold and self.hits >= 5:
            self.classification = 'static'
        elif speed >= speed_threshold and self.hits >= 3:
            self.classification = 'moving'
        else:
            self.classification = 'unknown'
            
    def to_track(self) -> Track:
        """Convert to Track dataclass."""
        return Track(
            track_id=self.track_id,
            position=self.get_position(),
            velocity=self.get_velocity(),
            covariance=self.P.copy(),
            timestamp=0.0,  # Caller should set
            age=self.age,
            hits=self.hits,
            classification=self.classification
        )


class Tracker:
    """
    Multi-object tracker for moving obstacles.
    
    Implements:
    - Clustering of detections
    - Data association (nearest-neighbor with gating)
    - Track management (creation, update, deletion)
    - Static/moving classification
    """
    
    def __init__(self, config: dict):
        self.config = config
        self.tracks = []
        self.next_id = 0
        
        # Parameters
        self.cluster_distance = config.get('tracker_cluster_distance', 0.5)
        self.gate_threshold = config.get('tracker_gate_threshold', 4.0)  # Chi-square for 2 DOF
        self.max_age = config.get('tracker_max_age', 5)  # Frames without update before deletion
        self.min_hits = config.get('tracker_min_hits', 3)  # Min hits to be considered valid
        self.speed_threshold = config.get('tracker_speed_threshold', 0.3)
        self.dt = config.get('timestep', 0.1)
        
    def update(self, detections: np.ndarray) -> List[Track]:
        """
        Update tracker with new detections.
        
        Args:
            detections: Nx2 array of detected obstacle positions [x, y]
            
        Returns:
            List of valid tracks
        """
        # Cluster detections
        if len(detections) > 0:
            clusters = cluster_points(detections, self.cluster_distance)
            # Use cluster centroids as detections
            det_centroids = [np.mean(cluster, axis=0) for cluster in clusters]
        else:
            det_centroids = []
            
        # Predict all tracks
        for track in self.tracks:
            track.predict()
            
        # Associate detections to tracks
        if len(self.tracks) > 0 and len(det_centroids) > 0:
            matched, unmatched_tracks, unmatched_dets = self._associate(det_centroids)
            
            # Update matched tracks
            for track_idx, det_idx in matched:
                self.tracks[track_idx].update(det_centroids[det_idx])
                self.tracks[track_idx].classify(self.speed_threshold)
                
            # Create new tracks for unmatched detections
            for det_idx in unmatched_dets:
                new_track = CVKalmanTrack(det_centroids[det_idx], self.next_id, self.dt)
                self.next_id += 1
                self.tracks.append(new_track)
                
        elif len(det_centroids) > 0:
            # No existing tracks, create new for all detections
            for det in det_centroids:
                new_track = CVKalmanTrack(det, self.next_id, self.dt)
                self.next_id += 1
                self.tracks.append(new_track)
                
        # Delete old tracks
        self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_age]
        
        # Return valid tracks (enough hits)
        valid_tracks = [t.to_track() for t in self.tracks if t.hits >= self.min_hits]
        return valid_tracks
        
    def _associate(self, detections: List[np.ndarray]) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        """
        Associate detections to tracks using nearest-neighbor with gating.
        
        Returns:
            matched: List of (track_idx, det_idx) pairs
            unmatched_tracks: List of track indices
            unmatched_dets: List of detection indices
        """
        if len(self.tracks) == 0:
            return [], [], list(range(len(detections)))
        if len(detections) == 0:
            return [], list(range(len(self.tracks))), []
            
        # Compute cost matrix (Mahalanobis distance)
        cost = np.zeros((len(self.tracks), len(detections)))
        for i, track in enumerate(self.tracks):
            for j, det in enumerate(detections):
                # Predicted measurement
                z_pred = track.H @ track.x
                y = det - z_pred
                
                # Innovation covariance
                S = track.H @ track.P @ track.H.T + track.R
                
                # Mahalanobis distance
                dist = y.T @ np.linalg.solve(S, y)
                cost[i, j] = dist
                
        # Greedy assignment with gating
        matched = []
        unmatched_tracks = list(range(len(self.tracks)))
        unmatched_dets = list(range(len(detections)))
        
        while len(unmatched_tracks) > 0 and len(unmatched_dets) > 0:
            # Find minimum cost
            min_cost = np.inf
            min_i, min_j = -1, -1
            for i in unmatched_tracks:
                for j in unmatched_dets:
                    if cost[i, j] < min_cost:
                        min_cost = cost[i, j]
                        min_i, min_j = i, j
                        
            # Check gate
            if min_cost > self.gate_threshold:
                break
                
            # Match
            matched.append((min_i, min_j))
            unmatched_tracks.remove(min_i)
            unmatched_dets.remove(min_j)
            
        return matched, unmatched_tracks, unmatched_dets
        
    def predict_ttc(self, robot_pos: np.ndarray, robot_vel: np.ndarray, track: Track) -> Optional[float]:
        """
        Predict Time To Collision with a track.
        
        Assumes constant velocity for both robot and obstacle.
        
        Returns:
            TTC in seconds, or None if no collision predicted
        """
        # Relative position and velocity
        rel_pos = track.position - robot_pos
        rel_vel = track.velocity - robot_vel
        
        # Check if approaching
        if np.dot(rel_pos, rel_vel) >= 0:
            return None  # Moving apart
            
        # TTC = -distance / closing_speed
        distance = np.linalg.norm(rel_pos)
        closing_speed = -np.dot(rel_pos, rel_vel) / distance
        
        if closing_speed <= 0:
            return None
            
        ttc = distance / closing_speed
        return ttc


def test_tracker():
    """Unit tests for tracker."""
    config = {
        'tracker_cluster_distance': 0.5,
        'tracker_gate_threshold': 4.0,
        'tracker_max_age': 5,
        'tracker_min_hits': 2,
        'tracker_speed_threshold': 0.3,
        'timestep': 0.1
    }
    
    tracker = Tracker(config)
    
    # Test 1: Create track from detection
    detections = np.array([[1.0, 2.0]])
    tracks = tracker.update(detections)
    assert len(tracker.tracks) == 1  # Created but not valid yet (min_hits=2)
    print(f"✓ Track created, age={tracker.tracks[0].age}, hits={tracker.tracks[0].hits}")
    
    # Test 2: Update with same detection (should associate)
    detections = np.array([[1.05, 2.05]])
    tracks = tracker.update(detections)
    assert len(tracker.tracks) == 1
    assert tracker.tracks[0].hits == 2
    assert len(tracks) == 1  # Now valid
    print(f"✓ Track updated, hits={tracker.tracks[0].hits}, pos={tracks[0].position}")
    
    # Test 3: Moving obstacle (velocity estimation)
    for i in range(5):
        x = 1.0 + 0.2 * (i+3)  # Moving at 2 m/s
        y = 2.0
        detections = np.array([[x, y]])
        tracks = tracker.update(detections)
        
    assert len(tracks) == 1
    vx_est = tracks[0].velocity[0]
    assert 1.5 <= vx_est <= 2.5  # Should estimate ~2 m/s
    assert tracks[0].classification == 'moving'
    print(f"✓ Moving obstacle: v={tracks[0].velocity}, speed={np.linalg.norm(tracks[0].velocity):.2f} m/s, class={tracks[0].classification}")
    
    # Test 4: Track deletion (no detections)
    for i in range(6):
        tracks = tracker.update(np.array([]))
    assert len(tracker.tracks) == 0  # Should be deleted after max_age
    print("✓ Track deleted after max_age")
    
    # Test 5: Multiple tracks
    detections = np.array([[0.0, 0.0], [5.0, 5.0]])
    tracker = Tracker(config)  # Reset
    for i in range(3):
        tracks = tracker.update(detections + 0.01*i)  # Slight movement
    assert len(tracks) == 2
    print(f"✓ Multiple tracks: {len(tracks)} tracks")
    
    # Test 6: TTC
    robot_pos = np.array([0.0, 0.0])
    robot_vel = np.array([0.0, 0.0])  # Stationary
    track = Track(
        track_id=0,
        position=np.array([5.0, 0.0]),
        velocity=np.array([-1.0, 0.0]),  # Approaching at 1 m/s
        covariance=np.eye(4),
        timestamp=0.0,
        age=10,
        hits=10,
        classification='moving'
    )
    ttc = tracker.predict_ttc(robot_pos, robot_vel, track)
    assert ttc is not None and 4.5 <= ttc <= 5.5
    print(f"✓ TTC: {ttc:.2f} s")
    
    print("✓ All tracker tests passed")


if __name__ == "__main__":
    test_tracker()
