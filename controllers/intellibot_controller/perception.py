"""perception.py — Webots device wrappers (the ONLY file besides the entry
point that touches the Webots API, apart from drive_test).

Wraps: GPS + IMU pose, wheel encoders + odometry, LiDAR scan (with the
robot self-filter), front + rear camera channels -> OpenCV detections, motor
commands. Image preprocessing itself lives in camera_pipeline.py (pure).

Webots conventions verified against docs/webots_findings.md:
    * devices must be enable(timestep)-d before first use
    * first PositionSensor value may be NaN
    * camera.getImage() returns BGRA bytes valid until the next step
      (converted by camera_pipeline.bgra_to_bgr: drop alpha, no flip)
    * LiDAR ray angle mapping uses config.LIDAR_ANGLE_SIGN / _OFFSET
      (empirically measured; see Phase 4.2)
"""

import math

import config
import detector as detector_mod
import estimation
from camera_pipeline import FRONT, REAR, FrameHealthMonitor, make_frame
from kinematics import Odometry
from mapping import self_filter


class CameraChannel:
    """One physical camera bound to ONE role ("front" or "rear").

    The role comes from the device name in config (CAMERA_NAME ->
    front, REAR_CAMERA_NAME -> rear), is stored with every frame and with
    every detection, and is the only thing estimation.py uses to decide
    whether a bearing points forwards or backwards — so the two feeds can
    never be interchanged silently.
    """

    def __init__(self, robot, timestep, device_name, role):
        self.role = role
        self.name = device_name
        self.device = None
        self.width = self.height = 0
        self.fov = None
        try:
            self.device = robot.getDevice(device_name)
        except Exception:
            self.device = None
        if self.device is None:
            print(f"[PERCEPTION] camera '{device_name}' ({role}) not found")
            return
        self.device.enable(timestep)
        self.width = self.device.getWidth()
        self.height = self.device.getHeight()
        try:
            self.fov = self.device.getFov()
        except Exception:
            self.fov = (config.REAR_CAMERA_FOV if role == REAR else config.CAMERA_FOV)
        self.monitor = FrameHealthMonitor(role)
        if not (0.0 < self.fov < math.pi):
            print(f"[PERCEPTION] WARNING: {device_name} fieldOfView {self.fov:.4f} rad "
                  f"is not a valid planar FOV (0, pi) — image will be degenerate")

    @property
    def available(self):
        return self.device is not None

    def read(self, t=0.0, moving=True):
        """-> camera_pipeline.CameraFrame (valid=False until the first sample)."""
        if self.device is None:
            return make_frame(self.role, None, 0, 0, t)
        return make_frame(self.role, self.device.getImage(), self.width,
                          self.height, t, self.monitor, moving)


class Perception:
    def __init__(self, robot, timestep):
        self.robot = robot
        self.dt = timestep

        # motors
        self.left_motor = robot.getDevice(config.LEFT_MOTOR)
        self.right_motor = robot.getDevice(config.RIGHT_MOTOR)
        for m in (self.left_motor, self.right_motor):
            m.setPosition(float("inf"))          # velocity mode
        # encoders
        self.left_encoder = robot.getDevice(config.LEFT_ENCODER)
        self.right_encoder = robot.getDevice(config.RIGHT_ENCODER)
        self.left_encoder.enable(self.dt)
        self.right_encoder.enable(self.dt)
        self.odometry = Odometry()

        # lidar
        self.lidar = robot.getDevice(config.LIDAR_NAME)
        self.lidar.enable(self.dt)
        self.n_rays = self.lidar.getHorizontalResolution() or config.LIDAR_RAYS
        self.lidar_fov = self.lidar.getFov()
        self.lidar_max = self.lidar.getMaxRange()
        self._ray_angles = self._compute_ray_angles()

        # cameras: one channel per physical camera, each bound to its role
        self.front_cam = CameraChannel(robot, self.dt, config.CAMERA_NAME, FRONT)
        self.rear_cam = CameraChannel(robot, self.dt, config.REAR_CAMERA_NAME, REAR)
        # backwards-compatible attributes used by older code/tools
        self.camera = self.front_cam.device
        self.cam_w, self.cam_h = self.front_cam.width, self.front_cam.height
        self.rear_camera = self.rear_cam.device
        self.rear_cam_w, self.rear_cam_h = self.rear_cam.width, self.rear_cam.height
        self.last_frames = {}

        # gps + imu
        self.gps = robot.getDevice(config.GPS_NAME)
        self.gps.enable(self.dt)
        self.imu = robot.getDevice(config.IMU_NAME)
        self.imu.enable(self.dt)

    # ------------------------------------------------------------- ray angles
    def _compute_ray_angles(self):
        """Bearing (robot frame, left=+) of each LiDAR ray index.

        Measured convention: ray 0 at -fov/2 (rightmost), index increasing
        CCW toward +fov/2, unless webots_findings.md records otherwise.
        config.LIDAR_ANGLE_SIGN / _OFFSET carry the measured correction.
        """
        n = self.n_rays
        base = [-self.lidar_fov / 2.0 + i * self.lidar_fov / (n - 1)
                for i in range(n)]
        return [config.LIDAR_ANGLE_SIGN * a + config.LIDAR_ANGLE_OFFSET
                for a in base]

    def ray_angles(self):
        return self._ray_angles

    # ------------------------------------------------------------------ pose
    def get_pose(self):
        """Operating pose estimate (GPS + IMU): (x, y, theta)."""
        p = self.gps.getValues()
        rpy = self.imu.getRollPitchYaw()
        x = p[0] if (p is not None and len(p) >= 1 and math.isfinite(p[0])) else config.ROBOT_START[0]
        y = p[1] if (p is not None and len(p) >= 2 and math.isfinite(p[1])) else config.ROBOT_START[1]
        yaw = rpy[2] if (rpy is not None and len(rpy) >= 3 and math.isfinite(rpy[2])) else config.ROBOT_START[2]
        return x, y, yaw

    def get_odometry(self):
        """Update + return wheel odometry pose (validated against GPS)."""
        applied = self.odometry.update(self.left_encoder.getValue(),
                                       self.right_encoder.getValue())
        return self.odometry.pose(), applied

    # ----------------------------------------------------------------- lidar
    def get_lidar(self):
        """(ranges, angles): numpy arrays; invalid returns -> inf."""
        import numpy as np
        img = self.lidar.getRangeImage()
        if img is None:
            return (np.full(len(self._ray_angles), math.inf),
                    np.asarray(self._ray_angles))
        ranges = np.asarray(img, dtype=float)
        # keep validity semantics: inf = no return (already inf), NaN -> inf;
        # returns from the robot's own arm post are removed (self-filter)
        ranges = self_filter(ranges, config.LIDAR_SELF_FILTER_RANGE)
        return ranges, np.asarray(self._ray_angles)

    # ---------------------------------------------------------------- camera
    def get_camera_frames(self, t=0.0, moving=True):
        """Both cameras -> {"front": CameraFrame, "rear": CameraFrame}."""
        self.last_frames = {FRONT: self.front_cam.read(t, moving),
                            REAR: self.rear_cam.read(t, moving)}
        return self.last_frames

    def get_frame(self):
        """FRONT camera BGR ndarray (alpha dropped), or None if not valid yet."""
        return self.front_cam.read().image

    def get_rear_frame(self):
        """REAR camera BGR ndarray, or None if absent / not valid yet."""
        return self.rear_cam.read().image

    def get_detections(self, frame=None, rear_frame=None):
        """Detections from both cameras, each tagged with its source camera.

        frame / rear_frame: BGR arrays already read this step (optional).
        Invalid (None) frames are skipped instead of crashing the detector.
        """
        if frame is None:
            frame = self.get_frame()
        dets = []
        if frame is not None:
            dets.extend(detector_mod.detect_colors(frame, is_rear=False))
        if self.rear_cam.available:
            if rear_frame is None:
                rear_frame = self.get_rear_frame()
            if rear_frame is not None:
                dets.extend(detector_mod.detect_colors(rear_frame, is_rear=True))
        return dets

    def camera_for(self, detection):
        """The channel a detection came from (front unless tagged rear)."""
        return self.rear_cam if getattr(detection, "is_rear", False) else self.front_cam

    def estimate_package_position(self, detection, pose, scan, angles):
        cam = self.camera_for(detection)
        hfov = cam.fov if cam.fov else (config.REAR_CAMERA_FOV if cam.role == REAR
                                        else config.CAMERA_FOV)
        return estimation.estimate_package_position(
            detection, pose, scan, angles,
            width=cam.width or config.CAMERA_WIDTH, hfov=hfov)

    # ---------------------------------------------------------------- motors
    def set_wheel_speeds(self, w_left, w_right):
        self.left_motor.setVelocity(w_left)
        self.right_motor.setVelocity(w_right)


def check_world_consistency(supervisor):
    """Phase 2.3: compare DEF object positions against config. Warn on
    mismatch > WORLD_CHECK_TOL. Returns True if consistent."""
    if not config.WORLD_CHECK_ENABLED:
        return True
    ok = True
    import shelving
    checks = [(defname, x, y) for (defname, x, y, _c) in config.PACKAGES]
    checks += [(d, x, y) for (x, y, _sx, _sy, _h, d) in config.SHELVES]
    checks += [(d, x, y) for (x, y, _sx, _sy, _h, d) in config.CRATES]
    checks += [(r.def_name, r.center[0], r.center[1])
               for r in shelving.build_racks().values()]
    for defname, x_cfg, y_cfg in checks:
        node = supervisor.getFromDef(defname)
        if node is None:
            print(f"[WORLD CHECK] WARNING: DEF {defname} not found in world")
            ok = False
            continue
        pos = node.getPosition()
        if math.hypot(pos[0] - x_cfg, pos[1] - y_cfg) > config.WORLD_CHECK_TOL:
            print(f"[WORLD CHECK] WARNING: {defname} at "
                  f"({pos[0]:.3f}, {pos[1]:.3f}) but config says "
                  f"({x_cfg:.3f}, {y_cfg:.3f})")
            ok = False
    if ok:
        print("[WORLD CHECK] world matches config")
    return ok
