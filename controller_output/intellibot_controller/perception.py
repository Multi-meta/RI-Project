"""perception.py — Webots device wrappers (the ONLY file besides the entry
point that touches the Webots API, apart from drive_test).

Wraps: GPS + IMU pose, wheel encoders + odometry, LiDAR scan, camera frame
-> OpenCV detections, motor commands.

Webots conventions verified against docs/webots_findings.md:
    * devices must be enable(timestep)-d before first use
    * first PositionSensor value may be NaN
    * camera.getImage() returns BGRA bytes valid until the next step
    * LiDAR ray angle mapping uses config.LIDAR_ANGLE_SIGN / _OFFSET
      (empirically measured; see Phase 4.2)
"""

import math

import controller_output.intellibot_controller.config as config
import controller_output.intellibot_controller.detector as detector_mod
import controller_output.intellibot_controller.estimation as estimation
from controller_output.intellibot_controller.kinematics import Odometry


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

        # camera
        self.camera = robot.getDevice(config.CAMERA_NAME)
        self.camera.enable(self.dt)
        self.cam_w = self.camera.getWidth()
        self.cam_h = self.camera.getHeight()

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
        yaw = self.imu.getRollPitchYaw()[2]
        return p[0], p[1], yaw

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
        ranges = np.asarray(img, dtype=float)
        # keep validity semantics: inf = no return (already inf), NaN -> inf
        ranges[~np.isfinite(ranges)] = math.inf
        return ranges, np.asarray(self._ray_angles)

    # ---------------------------------------------------------------- camera
    def get_frame(self):
        """BGR ndarray copy (alpha dropped). Webots gives BGRA bytes."""
        import numpy as np
        data = self.camera.getImage()
        img = np.frombuffer(data, np.uint8).reshape(self.cam_h, self.cam_w, 4)
        return img[:, :, :3].copy()

    def get_detections(self, frame=None):
        if frame is None:
            frame = self.get_frame()
        return detector_mod.detect_colors(frame)

    def estimate_package_position(self, detection, pose, scan, angles):
        return estimation.estimate_package_position(
            detection, pose, scan, angles,
            width=self.cam_w, hfov=self.camera.getFov())

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
    checks = [(defname, x, y) for (defname, x, y, _c) in config.PACKAGES]
    checks += [(d, x, y) for (x, y, _sx, _sy, _h, d) in config.SHELVES]
    checks += [(d, x, y) for (x, y, _sx, _sy, _h, d) in config.CRATES]
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
