"""camera_pipeline.py — camera image preprocessing and frame health (pure).

Webots -> OpenCV contract (verified against the Webots reference manual):
    * Camera.getImage() returns a bytes buffer of width*height*4 bytes in
      BGRA order, rows top -> bottom, columns left -> right — the same memory
      layout OpenCV uses for BGR(A) images. Conversion is therefore ONLY
      "drop the alpha channel": no channel swap, no flip, no transpose.
    * The buffer is only valid until the next robot.step(), so it is copied.
    * Before the first sampling period after enable() the image may be None
      or empty -> reported as an invalid frame, never passed to the detector.

The rear camera is the same device type rotated by pi about z: its image is
what you would see standing behind the robot looking backwards — NOT a mirror
image — so image-right is robot-LEFT (+y). estimation.bearing_from_detection
handles that mapping; this module never flips images.

Frame health (independent per camera, so a bad rear feed can never
silently stand in for the front feed):
    BLANK  : grey-level standard deviation < CAMERA_BLANK_STD_MIN (lens
             covered, lights off, degenerate field of view, ...)
    DARK   : grey-level mean < CAMERA_DARK_MEAN (lights off): bearings from
             this camera get infinite variance (camera_bearing_variance)
    LOW_CONTRAST : grey std < CAMERA_LOW_CONTRAST_STD (smoke / fog)
    FROZEN : CAMERA_FROZEN_FRAMES identical consecutive frames
"""

import zlib
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

import config

FRONT = "front"
REAR = "rear"


class CameraFrameError(ValueError):
    """Raised for a buffer that cannot be a width x height BGRA image."""


def bgra_to_bgr(buffer, width: int, height: int) -> np.ndarray:
    """Webots BGRA bytes -> contiguous BGR uint8 array (height, width, 3).

    Raises CameraFrameError for None / wrong-length buffers.
    """
    if buffer is None:
        raise CameraFrameError("no image buffer (camera not sampled yet?)")
    if width <= 0 or height <= 0:
        raise CameraFrameError(f"invalid image size {width}x{height}")
    expected = width * height * 4
    if isinstance(buffer, np.ndarray):
        arr = buffer.astype(np.uint8, copy=False).reshape(-1)
    else:
        arr = np.frombuffer(bytes(buffer), dtype=np.uint8)
    if arr.size != expected:
        raise CameraFrameError(f"buffer has {arr.size} bytes, expected {expected} "
                               f"for {width}x{height} BGRA")
    return np.ascontiguousarray(arr.reshape(height, width, 4)[:, :, :3])


def frame_stats(bgr: np.ndarray) -> dict:
    """Brightness / contrast summary used for health checks and logging.

    Computed on every 2nd pixel in float32: the full-resolution float64 copy
    (2 x 600 KiB per frame, 60 frames/s per camera) was pure allocation churn,
    and was where the controller died when Webots had exhausted the RAM."""
    sub = bgr[::2, ::2]
    grey = sub.mean(axis=2, dtype=np.float32)
    return {"mean": float(grey.mean()), "std": float(grey.std()),
            "min": int(sub.min()), "max": int(sub.max())}


@dataclass
class CameraFrame:
    """One preprocessed frame, tagged with the camera it came from."""
    source: str                       # "front" | "rear"
    image: Optional[np.ndarray]       # BGR, or None when invalid
    t: float
    valid: bool
    health: str = "OK"                # OK | BLANK | FROZEN | INVALID
    reason: str = ""
    stats: dict = field(default_factory=dict)


class FrameHealthMonitor:
    """Tracks blank/frozen frames for ONE camera."""

    def __init__(self, source: str, blank_std=None, frozen_frames=None):
        self.source = source
        self.blank_std = config.CAMERA_BLANK_STD_MIN if blank_std is None else blank_std
        self.frozen_frames = (config.CAMERA_FROZEN_FRAMES if frozen_frames is None
                              else frozen_frames)
        self._last_crc = None
        self._same = 0

    def check(self, bgr: np.ndarray, moving: bool = True):
        """-> (health, reason, stats).

        moving: the robot base is moving. A simulated camera looking at a
        static scene from a standing robot returns bit-identical frames, so
        identical frames only count towards FROZEN while the base moves
        (this produced false OK -> FROZEN flapping at every stop)."""
        stats = frame_stats(bgr)
        crc = zlib.crc32(bgr.tobytes())
        same = crc == self._last_crc
        self._same = self._same + 1 if (same and moving) else 0
        self._last_crc = crc
        if stats["std"] < self.blank_std:
            return "BLANK", f"grey std {stats['std']:.2f} < {self.blank_std}", stats
        if stats["mean"] < config.CAMERA_DARK_MEAN:
            return "DARK", f"grey mean {stats['mean']:.1f} < {config.CAMERA_DARK_MEAN}", stats
        if stats["std"] < config.CAMERA_LOW_CONTRAST_STD:
            return "LOW_CONTRAST", f"grey std {stats['std']:.1f} < {config.CAMERA_LOW_CONTRAST_STD}", stats
        if self._same >= self.frozen_frames:
            return "FROZEN", f"{self._same + 1} identical frames", stats
        return "OK", "", stats


def make_frame(source: str, buffer, width: int, height: int, t: float,
               monitor: Optional[FrameHealthMonitor] = None,
               moving: bool = True) -> CameraFrame:
    """Buffer -> CameraFrame (never raises; invalid input -> valid=False)."""
    try:
        bgr = bgra_to_bgr(buffer, width, height)
    except CameraFrameError as exc:
        return CameraFrame(source, None, t, False, "INVALID", str(exc))
    if monitor is None:
        return CameraFrame(source, bgr, t, True, "OK", "", frame_stats(bgr))
    try:
        health, reason, stats = monitor.check(bgr, moving)
    except MemoryError:                 # never take the controller down
        return CameraFrame(source, bgr, t, True, "INVALID", "health check: out of memory")
    # a blank/frozen frame is still a real image (e.g. lights off) — it is
    # passed on but flagged, so the health layer can de-weight the camera
    return CameraFrame(source, bgr, t, True, health, reason, stats)


def camera_bearing_variance(mean_brightness: float, base_var=None) -> float:
    """Measurement variance for a camera bearing, scaled by image brightness.

    var = base_var * (CAMERA_NOMINAL_MEAN / mean)^2   for mean < nominal
        = base_var                                    for mean >= nominal
        = inf                                         for mean < CAMERA_DARK_MEAN
    (inf -> ekf_localization.update_bearing skips the update). This is the
    "R scaled by light level" policy of checklist 11.5: in the dark the
    camera's Kalman gain shrinks smoothly and then the camera is ignored.
    """
    if base_var is None:
        base_var = config.EKF_R_CAMERA_BEARING
    if mean_brightness is None or not (mean_brightness >= config.CAMERA_DARK_MEAN):
        return float("inf")
    if mean_brightness >= config.CAMERA_NOMINAL_MEAN:
        return float(base_var)
    return float(base_var * (config.CAMERA_NOMINAL_MEAN / mean_brightness) ** 2)
