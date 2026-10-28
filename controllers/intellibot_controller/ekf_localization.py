"""
EKF localization — Extended Kalman Filter for sensor fusion.

State: [x, y, theta]
Inputs: odometry increments (δs, δθ from wheel encoders)
Updates: GPS (x,y), IMU (θ), landmarks (range-bearing)

Per addendum §A6.5, uses Joseph form for covariance update, 
angle-wrapped innovations, and χ² gating.
"""

import numpy as np
from typing import Tuple, Optional
from kinematics import wrap_to_pi


class EKFLocalizer:
    """
    Extended Kalman Filter for 2D robot localization.
    
    State: x = [x, y, theta]'
    Process model: x_{k+1} = f(x_k, u_k) + w, where u = [δs, δθ]
    """
    
    def __init__(self, x0: np.ndarray, P0: np.ndarray, Q: np.ndarray):
        """
        Args:
            x0: Initial state [x, y, theta]
            P0: Initial covariance 3x3
            Q: Process noise covariance 3x3
        """
        self.x = x0.copy()
        self.P = P0.copy()
        self.Q = Q.copy()
        
        # Statistics
        self.innovation_history = []
        self.nis_history = []
        
    def predict(self, ds: float, dtheta: float):
        """
        Prediction step from odometry increment.
        
        Args:
            ds: Distance traveled (m)
            dtheta: Rotation (rad)
        """
        # Current state
        x, y, theta = self.x
        
        # Nonlinear motion model
        theta_mid = theta + 0.5 * dtheta  # Midpoint integration
        x_new = x + ds * np.cos(theta_mid)
        y_new = y + ds * np.sin(theta_mid)
        theta_new = wrap_to_pi(theta + dtheta)
        
        # Jacobian of motion model w.r.t. state
        # f(x) = [x + ds*cos(theta + dtheta/2), y + ds*sin(theta + dtheta/2), theta + dtheta]
        # ∂f/∂x = I + [0 0 -ds*sin(theta_mid); 0 0 ds*cos(theta_mid); 0 0 0]
        F = np.eye(3)
        F[0, 2] = -ds * np.sin(theta_mid)
        F[1, 2] = ds * np.cos(theta_mid)
        
        # Predicted state
        self.x = np.array([x_new, y_new, theta_new])
        
        # Predicted covariance
        self.P = F @ self.P @ F.T + self.Q
        
    def update_gps(self, z_gps: np.ndarray, R_gps: np.ndarray, gate_threshold: float = 9.21):
        """
        Update with GPS measurement z = [x, y].
        
        Args:
            z_gps: GPS measurement [x, y]
            R_gps: Measurement noise covariance 2x2
            gate_threshold: χ² threshold for gating (9.21 for 2 DOF at 99%)
            
        Returns:
            accepted: bool, whether update was accepted
        """
        # Measurement model: h(x) = [x, y]
        H = np.array([[1, 0, 0],
                      [0, 1, 0]])
        
        # Innovation
        z_pred = H @ self.x
        y = z_gps - z_pred
        
        # Innovation covariance
        S = H @ self.P @ H.T + R_gps
        
        # Mahalanobis distance (NIS)
        nis = y.T @ np.linalg.solve(S, y)
        self.nis_history.append(nis)
        
        # Gating
        if nis > gate_threshold:
            print(f"GPS update REJECTED: NIS={nis:.2f} > {gate_threshold:.2f}")
            return False
            
        # Kalman gain
        K = self.P @ H.T @ np.linalg.solve(S, np.eye(2))
        
        # State update
        self.x = self.x + K @ y
        self.x[2] = wrap_to_pi(self.x[2])
        
        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(3) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_gps @ K.T
        
        self.innovation_history.append(y)
        return True
        
    def update_imu(self, z_theta: float, R_theta: float, gate_threshold: float = 6.63):
        """
        Update with IMU yaw measurement z = theta.
        
        Args:
            z_theta: IMU yaw (rad)
            R_theta: Measurement noise variance (scalar)
            gate_threshold: χ² threshold for 1 DOF at 99%
            
        Returns:
            accepted: bool
        """
        # Measurement model: h(x) = theta
        H = np.array([[0, 0, 1]])
        
        # Innovation (angle-wrapped)
        z_pred = self.x[2]
        y = wrap_to_pi(z_theta - z_pred)
        
        # Innovation covariance
        S = H @ self.P @ H.T + R_theta
        
        # NIS
        nis = y**2 / S
        self.nis_history.append(nis)
        
        # Gating
        if nis > gate_threshold:
            print(f"IMU update REJECTED: NIS={nis:.2f} > {gate_threshold:.2f}")
            return False
            
        # Kalman gain
        K = self.P @ H.T / S
        K = K.reshape(-1)
        
        # State update
        self.x = self.x + K * y
        self.x[2] = wrap_to_pi(self.x[2])
        
        # Covariance update (Joseph form)
        I_KH = np.eye(3) - np.outer(K, H)
        self.P = I_KH @ self.P @ I_KH.T + np.outer(K, K) * R_theta
        
        return True
        
    def update_landmark(self, z: np.ndarray, landmark_pos: np.ndarray, R: np.ndarray,
                        gate_threshold: float = 9.21):
        """
        Update with landmark range-bearing measurement.
        
        Args:
            z: [range, bearing] to landmark
            landmark_pos: Known landmark position [lx, ly]
            R: Measurement noise 2x2
            gate_threshold: χ² for 2 DOF
            
        Returns:
            accepted: bool
        """
        x, y, theta = self.x
        lx, ly = landmark_pos
        
        # Predicted measurement
        dx = lx - x
        dy = ly - y
        q = dx**2 + dy**2
        r_pred = np.sqrt(q)
        bearing_pred = wrap_to_pi(np.arctan2(dy, dx) - theta)
        z_pred = np.array([r_pred, bearing_pred])
        
        # Innovation
        y_innov = z - z_pred
        y_innov[1] = wrap_to_pi(y_innov[1])  # Wrap bearing
        
        # Jacobian H
        sqrt_q = np.sqrt(q)
        H = np.array([
            [-dx/sqrt_q, -dy/sqrt_q, 0],
            [dy/q, -dx/q, -1]
        ])
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # NIS
        nis = y_innov.T @ np.linalg.solve(S, y_innov)
        self.nis_history.append(nis)
        
        # Gating
        if nis > gate_threshold:
            print(f"Landmark update REJECTED: NIS={nis:.2f} > {gate_threshold:.2f}")
            return False
            
        # Kalman gain
        K = self.P @ H.T @ np.linalg.solve(S, np.eye(2))
        
        # State update
        self.x = self.x + K @ y_innov
        self.x[2] = wrap_to_pi(self.x[2])
        
        # Covariance update (Joseph form)
        I_KH = np.eye(3) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        
        return True
        
    def get_pose(self) -> Tuple[float, float, float]:
        """Return current pose estimate."""
        return self.x[0], self.x[1], self.x[2]
        
    def get_covariance(self) -> np.ndarray:
        """Return current covariance."""
        return self.P.copy()
        
    def compute_nees(self, x_true: np.ndarray) -> float:
        """
        Compute Normalized Estimation Error Squared (NEES).
        
        NEES = (x - x_true)' P^{-1} (x - x_true)
        Should be ~ χ²(3) if filter is consistent.
        """
        error = self.x - x_true
        error[2] = wrap_to_pi(error[2])
        nees = error.T @ np.linalg.solve(self.P, error)
        return nees


def test_ekf():
    """Unit tests for EKF localization."""
    print("Testing EKF localization...")
    
    # Initial state and covariance
    x0 = np.array([0.0, 0.0, 0.0])
    P0 = np.diag([0.1**2, 0.1**2, (5*np.pi/180)**2])
    Q = np.diag([0.02**2, 0.02**2, (2*np.pi/180)**2])
    
    ekf = EKFLocalizer(x0, P0, Q)
    
    # Test 1: Predict forward 1m
    ekf.predict(ds=1.0, dtheta=0.0)
    x, y, theta = ekf.get_pose()
    assert abs(x - 1.0) < 0.01 and abs(y) < 0.01
    print(f"✓ Predict forward: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")
    
    # Test 2: GPS update
    z_gps = np.array([1.05, 0.02])
    R_gps = np.diag([0.1**2, 0.1**2])
    accepted = ekf.update_gps(z_gps, R_gps)
    assert accepted
    x, y, theta = ekf.get_pose()
    print(f"✓ GPS update: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")
    
    # Test 3: Predict turn
    ekf.predict(ds=0.5, dtheta=np.pi/4)
    x, y, theta = ekf.get_pose()
    assert abs(theta - np.pi/4) < 0.1
    print(f"✓ Predict turn: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")
    
    # Test 4: IMU update
    z_theta = np.pi/4 + 0.01
    R_theta = (3*np.pi/180)**2
    accepted = ekf.update_imu(z_theta, R_theta)
    assert accepted
    x, y, theta = ekf.get_pose()
    print(f"✓ IMU update: theta={theta:.3f}")
    
    # Test 5: Landmark update
    # Place a landmark at (5, 5), robot at ~(1.3, 0.3, pi/4)
    landmark = np.array([5.0, 5.0])
    dx_true = 5.0 - x
    dy_true = 5.0 - y
    r_true = np.sqrt(dx_true**2 + dy_true**2)
    bearing_true = wrap_to_pi(np.arctan2(dy_true, dx_true) - theta)
    z_landmark = np.array([r_true + 0.1, bearing_true + 0.05])
    R_landmark = np.diag([0.2**2, (10*np.pi/180)**2])
    accepted = ekf.update_landmark(z_landmark, landmark, R_landmark)
    assert accepted
    x, y, theta = ekf.get_pose()
    print(f"✓ Landmark update: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")
    
    # Test 6: Gating rejection
    z_bad = np.array([100.0, 200.0])  # Way off
    accepted = ekf.update_gps(z_bad, R_gps, gate_threshold=9.21)
    assert not accepted
    print("✓ Gating rejects outlier")
    
    # Test 7: NEES
    x_true = np.array([x + 0.05, y + 0.05, theta + 0.01])
    nees = ekf.compute_nees(x_true)
    print(f"✓ NEES: {nees:.2f} (expect ~ 3 for consistent filter)")
    
    print("✓ All EKF tests passed")


if __name__ == "__main__":
    test_ekf()
