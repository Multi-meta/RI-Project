"""
Mission KPI tracking and reporting.

Per addendum §A12, tracks:
- Bumper collision count
- Minimum clearance to obstacles
- Path length, time elapsed
- Replan count and reasons
- Sensor health events
- EKF RMSE (when available)
- Task success/failure counts
"""

import time
import json
import math
from typing import List, Dict, Optional
from dataclasses import dataclass, field, asdict
from enum import Enum


class ReplanReason(Enum):
    """Reasons for replanning."""
    BLOCKED_PATH = "blocked_path"
    DYNAMIC_OBSTACLE = "dynamic_obstacle"
    SENSOR_FAILURE = "sensor_failure"
    STUCK = "stuck"
    TIMEOUT = "timeout"
    OTHER = "other"


@dataclass
class KPISnapshot:
    """Snapshot of KPIs at a point in time."""
    timestamp: float
    
    # Collision metrics
    bumper_collisions: int = 0
    front_bumper_hits: int = 0
    rear_bumper_hits: int = 0
    
    # Clearance metrics
    min_clearance: float = float('inf')
    min_clearance_timestamp: float = 0.0
    current_clearance: float = float('inf')
    
    # Navigation metrics
    path_length: float = 0.0  # meters
    time_elapsed: float = 0.0  # seconds
    replan_count: int = 0
    replan_reasons: Dict[str, int] = field(default_factory=dict)
    
    # Localization metrics (if EKF is used)
    ekf_rmse_x: Optional[float] = None
    ekf_rmse_y: Optional[float] = None
    ekf_rmse_theta: Optional[float] = None
    
    # Sensor health events
    sensor_failures: int = 0
    sensor_degradations: int = 0
    
    # Task metrics
    tasks_completed: int = 0
    tasks_failed: int = 0
    
    # Recovery metrics
    recovery_attempts: int = 0
    recovery_successes: int = 0


class MissionKPITracker:
    """
    Tracks mission KPIs throughout robot operation.
    
    Usage:
        tracker = MissionKPITracker(robot_id="R1")
        
        # Each timestep:
        tracker.update_clearance(lidar_min_range)
        tracker.update_position(x, y)
        
        # On events:
        tracker.record_collision(is_front=True)
        tracker.record_replan(ReplanReason.BLOCKED_PATH)
        tracker.record_sensor_event(is_failure=True)
        
        # At mission end:
        report = tracker.generate_report()
        tracker.save_report("logs/mission_R1.json")
    """
    
    def __init__(self, robot_id: str = "R0"):
        self.robot_id = robot_id
        self.start_time = time.time()
        self.last_position = None
        
        # Current KPIs
        self.kpis = KPISnapshot(timestamp=self.start_time)
        
        # History for analysis
        self.clearance_history: List[tuple] = []  # (time, clearance)
        self.collision_events: List[dict] = []  # detailed collision records
        self.replan_events: List[dict] = []  # detailed replan records
        
    def update_clearance(self, clearance: float, timestamp: Optional[float] = None):
        """Update current clearance to nearest obstacle."""
        if timestamp is None:
            timestamp = time.time()
            
        self.kpis.current_clearance = clearance
        self.clearance_history.append((timestamp, clearance))
        
        if clearance < self.kpis.min_clearance:
            self.kpis.min_clearance = clearance
            self.kpis.min_clearance_timestamp = timestamp
            
    def update_position(self, x: float, y: float, timestamp: Optional[float] = None):
        """Update robot position and accumulate path length."""
        if timestamp is None:
            timestamp = time.time()
            
        if self.last_position is not None:
            dx = x - self.last_position[0]
            dy = y - self.last_position[1]
            dist = math.sqrt(dx*dx + dy*dy)
            self.kpis.path_length += dist
            
        self.last_position = (x, y)
        self.kpis.time_elapsed = timestamp - self.start_time
        
    def record_collision(self, is_front: bool = True, timestamp: Optional[float] = None,
                        velocity: Optional[float] = None, clearance: Optional[float] = None):
        """Record a bumper collision."""
        if timestamp is None:
            timestamp = time.time()
            
        self.kpis.bumper_collisions += 1
        if is_front:
            self.kpis.front_bumper_hits += 1
        else:
            self.kpis.rear_bumper_hits += 1
            
        self.collision_events.append({
            'timestamp': timestamp,
            'elapsed': timestamp - self.start_time,
            'bumper': 'front' if is_front else 'rear',
            'velocity': velocity,
            'clearance_before': clearance,
            'position': self.last_position
        })
        
    def record_replan(self, reason: ReplanReason, timestamp: Optional[float] = None,
                     old_path_remaining: Optional[float] = None):
        """Record a replan event."""
        if timestamp is None:
            timestamp = time.time()
            
        self.kpis.replan_count += 1
        reason_str = reason.value if isinstance(reason, ReplanReason) else str(reason)
        self.kpis.replan_reasons[reason_str] = self.kpis.replan_reasons.get(reason_str, 0) + 1
        
        self.replan_events.append({
            'timestamp': timestamp,
            'elapsed': timestamp - self.start_time,
            'reason': reason_str,
            'old_path_remaining': old_path_remaining,
            'position': self.last_position
        })
        
    def record_sensor_event(self, is_failure: bool, sensor_name: Optional[str] = None,
                           timestamp: Optional[float] = None):
        """Record a sensor health event."""
        if timestamp is None:
            timestamp = time.time()
            
        if is_failure:
            self.kpis.sensor_failures += 1
        else:
            self.kpis.sensor_degradations += 1
            
    def record_recovery(self, success: bool, timestamp: Optional[float] = None):
        """Record a recovery behavior attempt."""
        if timestamp is None:
            timestamp = time.time()
            
        self.kpis.recovery_attempts += 1
        if success:
            self.kpis.recovery_successes += 1
            
    def record_task(self, success: bool, task_type: Optional[str] = None,
                   timestamp: Optional[float] = None):
        """Record task completion or failure."""
        if timestamp is None:
            timestamp = time.time()
            
        if success:
            self.kpis.tasks_completed += 1
        else:
            self.kpis.tasks_failed += 1
            
    def update_ekf_error(self, error_x: float, error_y: float, error_theta: float):
        """Update EKF RMSE metrics (computed externally from ground truth)."""
        self.kpis.ekf_rmse_x = error_x
        self.kpis.ekf_rmse_y = error_y
        self.kpis.ekf_rmse_theta = error_theta
        
    def generate_report(self) -> dict:
        """Generate comprehensive mission report."""
        self.kpis.timestamp = time.time()
        self.kpis.time_elapsed = self.kpis.timestamp - self.start_time
        
        report = {
            'robot_id': self.robot_id,
            'mission_start': self.start_time,
            'mission_end': self.kpis.timestamp,
            'kpis': asdict(self.kpis),
            'summary': self._generate_summary(),
            'events': {
                'collisions': self.collision_events,
                'replans': self.replan_events
            }
        }
        
        return report
        
    def _generate_summary(self) -> dict:
        """Generate human-readable summary statistics."""
        summary = {
            'duration_minutes': self.kpis.time_elapsed / 60.0,
            'average_speed_mps': self.kpis.path_length / max(self.kpis.time_elapsed, 1.0),
            'collision_rate_per_meter': self.kpis.bumper_collisions / max(self.kpis.path_length, 1.0),
            'replan_rate_per_meter': self.kpis.replan_count / max(self.kpis.path_length, 1.0),
            'task_success_rate': self.kpis.tasks_completed / max(self.kpis.tasks_completed + self.kpis.tasks_failed, 1),
            'recovery_success_rate': self.kpis.recovery_successes / max(self.kpis.recovery_attempts, 1) if self.kpis.recovery_attempts > 0 else 1.0,
            'min_clearance_meters': self.kpis.min_clearance if self.kpis.min_clearance != float('inf') else None
        }
        return summary
        
    def save_report(self, filepath: str):
        """Save mission report to JSON file."""
        report = self.generate_report()
        with open(filepath, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"Mission report saved to {filepath}")
        
    def print_summary(self):
        """Print KPI summary to console."""
        summary = self._generate_summary()
        print(f"\n{'='*60}")
        print(f"Mission KPI Summary - Robot {self.robot_id}")
        print(f"{'='*60}")
        print(f"Duration:          {summary['duration_minutes']:.1f} min")
        print(f"Path length:       {self.kpis.path_length:.1f} m")
        print(f"Average speed:     {summary['average_speed_mps']:.2f} m/s")
        print(f"")
        print(f"Collisions:        {self.kpis.bumper_collisions} ({self.kpis.front_bumper_hits} front, {self.kpis.rear_bumper_hits} rear)")
        print(f"Collision rate:    {summary['collision_rate_per_meter']:.4f} per meter")
        print(f"Min clearance:     {self.kpis.min_clearance:.3f} m" if self.kpis.min_clearance != float('inf') else "Min clearance:     N/A")
        print(f"")
        print(f"Replans:           {self.kpis.replan_count}")
        print(f"Replan reasons:    {self.kpis.replan_reasons}")
        print(f"")
        print(f"Tasks completed:   {self.kpis.tasks_completed}")
        print(f"Tasks failed:      {self.kpis.tasks_failed}")
        print(f"Success rate:      {summary['task_success_rate']:.1%}")
        print(f"")
        print(f"Sensor failures:   {self.kpis.sensor_failures}")
        print(f"Sensor degraded:   {self.kpis.sensor_degradations}")
        print(f"")
        print(f"Recovery attempts: {self.kpis.recovery_attempts}")
        print(f"Recovery success:  {summary['recovery_success_rate']:.1%}")
        
        if self.kpis.ekf_rmse_x is not None:
            print(f"")
            print(f"EKF RMSE:          x={self.kpis.ekf_rmse_x:.3f}m, y={self.kpis.ekf_rmse_y:.3f}m, θ={self.kpis.ekf_rmse_theta:.3f}rad")
        print(f"{'='*60}\n")


# ============================================================================
# UNIT TESTS
# ============================================================================

def test_clearance_tracking():
    """Test clearance updates and minimum tracking."""
    tracker = MissionKPITracker("TEST")
    
    tracker.update_clearance(1.5, 100.0)
    assert tracker.kpis.current_clearance == 1.5
    assert tracker.kpis.min_clearance == 1.5
    
    tracker.update_clearance(0.3, 101.0)
    assert tracker.kpis.min_clearance == 0.3
    assert tracker.kpis.min_clearance_timestamp == 101.0
    
    tracker.update_clearance(2.0, 102.0)
    assert tracker.kpis.min_clearance == 0.3  # still minimum
    
    print("✓ Clearance tracking works")


def test_path_length():
    """Test path length accumulation."""
    tracker = MissionKPITracker("TEST")
    
    tracker.update_position(0.0, 0.0, 100.0)
    tracker.update_position(1.0, 0.0, 101.0)
    assert abs(tracker.kpis.path_length - 1.0) < 0.01
    
    tracker.update_position(1.0, 1.0, 102.0)
    assert abs(tracker.kpis.path_length - 2.0) < 0.01
    
    tracker.update_position(0.0, 1.0, 103.0)
    assert abs(tracker.kpis.path_length - 3.0) < 0.01
    
    print("✓ Path length accumulation works")


def test_collision_recording():
    """Test collision event recording."""
    tracker = MissionKPITracker("TEST")
    
    tracker.record_collision(is_front=True, velocity=0.5, clearance=0.1)
    assert tracker.kpis.bumper_collisions == 1
    assert tracker.kpis.front_bumper_hits == 1
    assert tracker.kpis.rear_bumper_hits == 0
    
    tracker.record_collision(is_front=False)
    assert tracker.kpis.bumper_collisions == 2
    assert tracker.kpis.rear_bumper_hits == 1
    
    assert len(tracker.collision_events) == 2
    assert tracker.collision_events[0]['bumper'] == 'front'
    assert tracker.collision_events[1]['bumper'] == 'rear'
    
    print("✓ Collision recording works")


def test_replan_recording():
    """Test replan event recording."""
    tracker = MissionKPITracker("TEST")
    
    tracker.record_replan(ReplanReason.BLOCKED_PATH, old_path_remaining=5.0)
    assert tracker.kpis.replan_count == 1
    assert tracker.kpis.replan_reasons['blocked_path'] == 1
    
    tracker.record_replan(ReplanReason.BLOCKED_PATH)
    tracker.record_replan(ReplanReason.DYNAMIC_OBSTACLE)
    
    assert tracker.kpis.replan_count == 3
    assert tracker.kpis.replan_reasons['blocked_path'] == 2
    assert tracker.kpis.replan_reasons['dynamic_obstacle'] == 1
    
    print("✓ Replan recording works")


def test_report_generation():
    """Test mission report generation."""
    tracker = MissionKPITracker("TEST")
    tracker.start_time = 100.0
    
    tracker.update_position(0.0, 0.0, 100.0)
    tracker.update_position(10.0, 0.0, 110.0)
    tracker.update_clearance(0.5, 105.0)
    tracker.record_collision(is_front=True)
    tracker.record_replan(ReplanReason.BLOCKED_PATH)
    tracker.record_task(success=True)
    tracker.record_task(success=True)
    tracker.record_task(success=False)
    tracker.update_ekf_error(0.05, 0.03, 0.01)
    
    report = tracker.generate_report()
    
    assert report['robot_id'] == "TEST"
    assert report['kpis']['bumper_collisions'] == 1
    assert report['kpis']['replan_count'] == 1
    assert report['kpis']['tasks_completed'] == 2
    assert report['kpis']['tasks_failed'] == 1
    assert abs(report['kpis']['path_length'] - 10.0) < 0.01
    assert report['kpis']['ekf_rmse_x'] == 0.05
    assert report['summary']['task_success_rate'] == 2/3
    
    print("✓ Report generation works")


def test_summary_output():
    """Test summary statistics computation."""
    tracker = MissionKPITracker("TEST")
    tracker.start_time = time.time() - 60.0  # 1 minute ago
    
    tracker.update_position(0.0, 0.0)
    tracker.update_position(50.0, 0.0)
    tracker.record_collision(is_front=True)
    tracker.record_replan(ReplanReason.STUCK)
    tracker.record_recovery(success=True)
    tracker.record_task(success=True)
    
    tracker.print_summary()
    
    summary = tracker._generate_summary()
    assert summary['average_speed_mps'] > 0
    assert summary['collision_rate_per_meter'] > 0
    assert summary['task_success_rate'] == 1.0
    
    print("✓ Summary computation works")


if __name__ == "__main__":
    print("Testing mission_kpis.py...")
    test_clearance_tracking()
    test_path_length()
    test_collision_recording()
    test_replan_recording()
    test_report_generation()
    test_summary_output()
    print("\n✅ All mission KPI tests passed!")
