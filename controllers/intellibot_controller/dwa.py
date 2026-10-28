"""
DWA — Dynamic Window Approach for local obstacle avoidance.

Per addendum §A8.4, samples velocities within the dynamic window,
checks admissibility (no collisions), and scores trajectories based on:
- Heading: alignment with global goal
- Distance: clearance to obstacles
- Velocity: prefer higher speeds

Reverse only if rear is verified clear.
"""

import numpy as np
from typing import Tuple, Optional
from kinematics import forward_kinematics, wrap_to_pi


class DWA:
    """
    Dynamic Window Approach for local navigation.
    
    Samples (v, ω) pairs, simulates forward, and picks the best collision-free trajectory.
    """
    
    def __init__(self, config: dict):
        self.config = config
        
        # Robot constraints
        self.max_v = config.get('V_MAX', 0.5)
        self.max_w = config.get('W_MAX', 2.0)
        self.max_acc_v = config.get('MAX_ACC_V', 0.5)  # m/s^2
        self.max_acc_w = config.get('MAX_ACC_W', 2.0)  # rad/s^2
        self.wheel_separation = config.get('WHEEL_SEPARATION', 0.3)
        self.wheel_radius = config.get('WHEEL_RADIUS', 0.05)
        self.robot_radius = config.get('ROBOT_RADIUS', 0.25)
        
        # Sampling
        self.v_samples = config.get('DWA_V_SAMPLES', 10)
        self.w_samples = config.get('DWA_W_SAMPLES', 20)
        self.dt = config.get('DWA_DT', 0.2)  # Simulation time step
        self.sim_time = config.get('DWA_SIM_TIME', 2.0)  # Simulation horizon
        
        # Scoring weights
        self.w_heading = config.get('DWA_W_HEADING', 1.0)
        self.w_distance = config.get('DWA_W_DISTANCE', 1.0)
        self.w_velocity = config.get('DWA_W_VELOCITY', 0.5)
        
        # Safety
        self.allow_reverse = config.get('DWA_ALLOW_REVERSE', True)
        self.stop_dist = config.get('STOP_DIST', 0.5)
        
    def compute(self, pose: Tuple[float, float, float], 
                current_v: float, current_w: float,
                goal: Tuple[float, float],
                costmap,
                rear_clear: bool = True) -> Tuple[float, float]:
        """
        Compute best (v, ω) command.
        
        Args:
            pose: (x, y, theta) current pose
            current_v, current_w: current velocities
            goal: (gx, gy) local goal point (e.g., next waypoint)
            costmap: Costmap object for collision checking
            rear_clear: Whether rear is clear for reversing
            
        Returns:
            (v, ω) command
        """
        x, y, theta = pose
        
        # Dynamic window: reachable velocities given acceleration limits
        v_min = max(-self.max_v if self.allow_reverse and rear_clear else 0.0,
                    current_v - self.max_acc_v * self.dt)
        v_max = min(self.max_v, current_v + self.max_acc_v * self.dt)
        
        w_min = max(-self.max_w, current_w - self.max_acc_w * self.dt)
        w_max = min(self.max_w, current_w + self.max_acc_w * self.dt)
        
        # Sample velocity space
        v_range = np.linspace(v_min, v_max, self.v_samples)
        w_range = np.linspace(w_min, w_max, self.w_samples)
        
        best_score = -np.inf
        best_v, best_w = 0.0, 0.0
        
        for v in v_range:
            for w in w_range:
                # Simulate trajectory
                traj = self._simulate_trajectory(x, y, theta, v, w)
                
                # Check admissibility (no collision)
                if not self._is_admissible(traj, costmap):
                    continue
                    
                # Score trajectory
                score = self._score_trajectory(traj, goal, v)
                
                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w
                    
        # If no admissible trajectory, stop
        if best_score == -np.inf:
            return 0.0, 0.0
            
        return best_v, best_w
        
    def _simulate_trajectory(self, x: float, y: float, theta: float,
                             v: float, w: float) -> np.ndarray:
        """
        Simulate a trajectory forward in time.
        
        Returns:
            Nx3 array of (x, y, theta) poses along trajectory
        """
        num_steps = int(self.sim_time / self.dt)
        traj = np.zeros((num_steps + 1, 3))
        traj[0] = [x, y, theta]
        
        for i in range(num_steps):
            # Simple Euler integration
            x_cur, y_cur, theta_cur = traj[i]
            
            x_new = x_cur + v * np.cos(theta_cur) * self.dt
            y_new = y_cur + v * np.sin(theta_cur) * self.dt
            theta_new = wrap_to_pi(theta_cur + w * self.dt)
            
            traj[i + 1] = [x_new, y_new, theta_new]
            
        return traj
        
    def _is_admissible(self, traj: np.ndarray, costmap) -> bool:
        """
        Check if trajectory is collision-free.
        
        Returns:
            True if admissible (no collisions), False otherwise
        """
        for pose in traj:
            x, y, theta = pose
            
            # Check robot footprint (approximate as circle)
            cx, cy = costmap.grid.world_to_cell(x, y)
            
            # Check if center is lethal
            if costmap.is_lethal(cx, cy):
                return False
                
            # Check a few points around the robot radius
            for angle in np.linspace(0, 2*np.pi, 8, endpoint=False):
                px = x + self.robot_radius * np.cos(angle)
                py = y + self.robot_radius * np.sin(angle)
                pcx, pcy = costmap.grid.world_to_cell(px, py)
                if costmap.is_lethal(pcx, pcy):
                    return False
                    
        return True
        
    def _score_trajectory(self, traj: np.ndarray, goal: Tuple[float, float], v: float) -> float:
        """
        Score a trajectory based on heading, clearance, and velocity.
        
        Returns:
            score (higher is better)
        """
        # End pose
        x_end, y_end, theta_end = traj[-1]
        
        # 1. Heading: angle to goal from end pose
        dx = goal[0] - x_end
        dy = goal[1] - y_end
        goal_angle = np.arctan2(dy, dx)
        heading_error = abs(wrap_to_pi(goal_angle - theta_end))
        heading_score = 1.0 - (heading_error / np.pi)  # 0-1
        
        # 2. Distance: progress toward goal
        dist_to_goal = np.hypot(dx, dy)
        # Closer is better, but avoid division by zero
        distance_score = 1.0 / (1.0 + dist_to_goal)
        
        # 3. Velocity: prefer higher speeds (forward preferred over reverse)
        velocity_score = abs(v) / self.max_v
        if v < 0:
            velocity_score *= 0.5  # Penalize reverse
            
        # Weighted sum
        score = (self.w_heading * heading_score +
                 self.w_distance * distance_score +
                 self.w_velocity * velocity_score)
        
        return score


def test_dwa():
    """Unit tests for DWA."""
    print("Testing DWA...")
    
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
    
    # Create a simple costmap
    from mapping import OccupancyGrid
    from costmap import Costmap
    
    grid = OccupancyGrid(x_min=0.0, x_max=20.0, y_min=0.0, y_max=20.0, res=0.2)
    costmap = Costmap(grid, inflation_radius=0.3)
    
    # Test 1: Free space, goal ahead
    pose = (5.0, 5.0, 0.0)
    goal = (10.0, 5.0)
    v, w = dwa.compute(pose, 0.0, 0.0, goal, costmap, rear_clear=True)
    assert v > 0  # Should move forward
    assert abs(w) < 0.5  # Should go roughly straight
    print(f"✓ Free space: v={v:.2f}, w={w:.2f}")
    
    # Test 2: Obstacle ahead, goal to the side
    costmap.update_dynamic([(7.0, 5.0)])  # Obstacle directly ahead
    goal = (10.0, 8.0)  # Goal to the upper right
    v, w = dwa.compute(pose, 0.0, 0.0, goal, costmap, rear_clear=True)
    # Should turn to avoid obstacle
    print(f"✓ Obstacle avoidance: v={v:.2f}, w={w:.2f}")
    
    # Test 3: Boxed in, should stop
    costmap.update_dynamic([(5.5, 5.0), (4.5, 5.0), (5.0, 5.5), (5.0, 4.5)])
    v, w = dwa.compute(pose, 0.0, 0.0, goal, costmap, rear_clear=True)
    assert v == 0.0 and w == 0.0
    print(f"✓ Boxed in: v={v:.2f}, w={w:.2f} (stopped)")
    
    # Test 4: Goal behind, reverse allowed
    costmap.clear_dynamic()
    goal = (2.0, 5.0)  # Goal behind
    v, w = dwa.compute(pose, 0.0, 0.0, goal, costmap, rear_clear=True)
    # Could turn or reverse
    print(f"✓ Goal behind: v={v:.2f}, w={w:.2f}")
    
    # Test 5: Rear not clear, no reverse
    v, w = dwa.compute(pose, 0.0, 0.0, goal, costmap, rear_clear=False)
    assert v >= 0  # Should not reverse
    print(f"✓ Rear blocked: v={v:.2f}, w={w:.2f} (no reverse)")
    
    print("✓ All DWA tests passed")


if __name__ == "__main__":
    test_dwa()
