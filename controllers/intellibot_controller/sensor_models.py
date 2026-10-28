"""
Sensor models — noise injection and wrapper API for sensor readings.

Per the addendum §A6, each sensor returns a SensorReading dataclass with
timestamp, valid flag, variance/quality. Noise is injected per config σ.
"""

import time
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class SensorReading:
    """Generic sensor reading with quality metadata."""
    timestamp: float
    valid: bool
    variance: Optional[float] = None
    quality: Optional[float] = None  # 0-1, sensor-specific


@dataclass
class Pose2D:
    """2D pose with covariance."""
    x: float
    y: float
    theta: float
    timestamp: float
    valid: bool
    covariance: Optional[np.ndarray] = None  # 3x3


@dataclass
class ObstacleReport:
    """Fused obstacle detection report."""
    sector: str  # 'front', 'left', 'right', 'rear', etc.
    range: float
    timestamp: float
    status: str  # 'VERIFIED', 'UNVERIFIED', 'CONFLICT'
    trust: float  # 0-1
    sources: list  # which sensors contributed


@dataclass
class Detection:
    """Vision detection with metadata (from detector.py but extended)."""
    color: str
    bbox: Tuple[int, int, int, int]
    score: float
    timestamp: float
    valid: bool


@dataclass
class Track:
    """Moving obstacle track."""
    track_id: int
    position: np.ndarray  # [x, y]
    velocity: np.ndarray  # [vx, vy]
    covariance: np.ndarray  # 4x4 [x, y, vx, vy]
    timestamp: float
    age: int  # frames since creation
    hits: int  # successful associations
    classification: str  # 'static', 'moving', 'unknown'


@dataclass
class Task:
    """Robot task assignment."""
    task_id: str
    task_type: str  # 'STORE', 'RETRIEVE'
    robot_id: int
    package_id: Optional[str]
    source: Tuple[float, float]
    destination: Tuple[float, float]
    slot_id: Optional[str]
    status: str  # 'ASSIGNED', 'IN_PROGRESS', 'COMPLETE', 'FAILED'
    timestamp: float


@dataclass
class Message:
    """Inter-robot communication message."""
    sender_id: int
    message_type: str
    payload: dict
    timestamp: float


class NoisyGPS:
    """GPS sensor wrapper with configurable noise."""
    
    def __init__(self, device, sigma_xy: float = 0.0):
        self.device = device
        self.sigma_xy = sigma_xy
        
    def read(self) -> Pose2D:
        """Return noisy GPS reading as Pose2D (x, y only, theta=0)."""
        raw = self.device.getValues()
        timestamp = time.time()
        
        # Inject noise
        noise = np.random.randn(2) * self.sigma_xy if self.sigma_xy > 0 else np.zeros(2)
        x = raw[0] + noise[0]
        y = raw[1] + noise[1]
        
        # Covariance
        cov = np.diag([self.sigma_xy**2, self.sigma_xy**2, 0.0])
        
        return Pose2D(x, y, 0.0, timestamp, True, cov)


class NoisyIMU:
    """InertialUnit sensor wrapper with configurable noise."""
    
    def __init__(self, device, sigma_yaw: float = 0.0):
        self.device = device
        self.sigma_yaw = sigma_yaw
        
    def read(self) -> float:
        """Return noisy yaw angle (radians)."""
        rpy = self.device.getRollPitchYaw()
        noise = np.random.randn() * self.sigma_yaw if self.sigma_yaw > 0 else 0.0
        return rpy[2] + noise


class NoisyLidar:
    """LiDAR sensor wrapper with configurable noise."""
    
    def __init__(self, device, sigma_range: float = 0.0):
        self.device = device
        self.sigma_range = sigma_range
        
    def read(self) -> SensorReading:
        """Return noisy range array."""
        raw = np.array(self.device.getRangeImage(), dtype=float)
        timestamp = time.time()
        
        # Inject range noise
        if self.sigma_range > 0:
            noise = np.random.randn(len(raw)) * self.sigma_range
            raw = raw + noise
            
        # Validity: inf means no return
        valid = np.isfinite(raw)
        
        reading = SensorReading(
            timestamp=timestamp,
            valid=np.any(valid),
            variance=self.sigma_range**2,
            quality=np.sum(valid) / len(valid)  # fraction of valid rays
        )
        
        return reading, raw, valid


class NoisyDistanceSensor:
    """IR/Sonar/Ultrasonic distance sensor wrapper with noise."""
    
    def __init__(self, device, sigma: float = 0.0):
        self.device = device
        self.sigma = sigma
        
    def read(self) -> SensorReading:
        """Return noisy distance reading."""
        raw = self.device.getValue()
        timestamp = time.time()
        
        # Check validity (sensor type dependent)
        valid = raw >= 0 and np.isfinite(raw)
        
        # Inject noise
        if valid and self.sigma > 0:
            noise = np.random.randn() * self.sigma
            value = raw + noise
        else:
            value = raw
            
        reading = SensorReading(
            timestamp=timestamp,
            valid=valid,
            variance=self.sigma**2 if valid else None,
            quality=1.0 if valid else 0.0
        )
        
        return reading, value


class NoisyCamera:
    """Camera sensor wrapper with quality assessment."""
    
    def __init__(self, device):
        self.device = device
        
    def read(self) -> SensorReading:
        """Return camera image with quality metrics."""
        timestamp = time.time()
        
        # Get raw image
        raw = self.device.getImage()
        if raw is None:
            return SensorReading(timestamp, False), None
            
        # Convert to numpy array
        width = self.device.getWidth()
        height = self.device.getHeight()
        img = np.frombuffer(raw, np.uint8).reshape((height, width, 4))
        bgr = img[:, :, [2, 1, 0]]  # Drop alpha, BGR order
        
        # Assess quality: brightness and contrast
        gray = 0.299 * bgr[:, :, 2] + 0.587 * bgr[:, :, 1] + 0.114 * bgr[:, :, 0]
        brightness = np.mean(gray) / 255.0
        contrast = np.std(gray) / 255.0
        
        # Quality heuristic: penalize very dark or very bright, reward contrast
        quality = 1.0
        if brightness < 0.2:  # very dark
            quality *= (brightness / 0.2)
        elif brightness > 0.9:  # very bright
            quality *= ((1.0 - brightness) / 0.1)
        quality *= min(1.0, contrast / 0.3)  # normalize contrast contribution
        
        reading = SensorReading(
            timestamp=timestamp,
            valid=True,
            variance=None,
            quality=quality
        )
        
        return reading, bgr, brightness, contrast


def test_sensor_models():
    """Unit tests for sensor models (requires mock devices)."""
    print("sensor_models.py: dataclass definitions and noise wrappers ready.")
    print("Tests require Webots device instances.")


if __name__ == "__main__":
    test_sensor_models()
