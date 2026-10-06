"""drive_test.py — Phase 2/3 validation controller (Webots).

Runs in worlds/warehouse_drive_test.wbt (same world as the mission, robot
controller = drive_test). Mode is chosen by the DRIVE_TEST_MODE env var:

    sequence    (default) forward, spin left, arc, reverse, stop; prints pose
    stability   motors at 0 for 10 s; logs drift/tilt (Phase 3.1)
    kinematics  commanded (v, w) vs measured from GPS/IMU per-step differences
                -> logs/kinematics_test.csv (Phase 3.3)
    odometry    line / square / arc; encoder odometry vs GPS+IMU
                -> logs/odometry.csv (Phase 3.4)
    worldcheck  supervisor DEF positions vs config.py (Phase 2.3)
    snapshot    top-down + angled screenshots -> docs/evidence/ (Phase 2.2)
    capture     spin in place like FIND_PACKAGE; save camera frames every 30 deg
                + print HSV detections -> docs/evidence/frames/spin_*.png

All output paths are relative to the repo root (not the controller CWD).
"""

import csv
import math
import os
import sys

# reuse the shared pure modules from the main controller folder. APPENDED (not
# inserted first): that folder has a controller.py which must not shadow the
# Webots `controller` API module.
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.append(os.path.join(_HERE, "..", "intellibot_controller"))

from controller import Supervisor  # noqa: E402  (Webots API)

import config  # noqa: E402
from kinematics import inverse_kinematics, Odometry, wrap_to_pi  # noqa: E402
from perception import check_world_consistency  # noqa: E402

LOG_DIR = os.path.join(_ROOT, "logs")
EVIDENCE_DIR = os.path.join(_ROOT, "docs", "evidence")


class DriveTest:
    def __init__(self):
        self.robot = Supervisor()
        self.dt = int(self.robot.getBasicTimeStep())
        self.left = self.robot.getDevice(config.LEFT_MOTOR)
        self.right = self.robot.getDevice(config.RIGHT_MOTOR)
        for m in (self.left, self.right):
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
        self.enc_l = self.robot.getDevice(config.LEFT_ENCODER)
        self.enc_r = self.robot.getDevice(config.RIGHT_ENCODER)
        self.enc_l.enable(self.dt)
        self.enc_r.enable(self.dt)
        self.gps = self.robot.getDevice(config.GPS_NAME)
        self.gps.enable(self.dt)
        self.imu = self.robot.getDevice(config.IMU_NAME)
        self.imu.enable(self.dt)
        self.odo = Odometry()
        self.step()                     # sensors valid from here on

    # ------------------------------------------------------------- helpers
    def step(self):
        return self.robot.step(self.dt) != -1

    def t(self):
        return self.robot.getTime()

    def pose(self):
        p = self.gps.getValues()
        return p[0], p[1], self.imu.getRollPitchYaw()[2]

    def cmd(self, v, w):
        wl, wr = inverse_kinematics(v, w)
        self.left.setVelocity(wl)
        self.right.setVelocity(wr)
        return wl, wr

    def stop(self):
        self.left.setVelocity(0.0)
        self.right.setVelocity(0.0)

    def hold(self, v, w, dur, on_step=None):
        """Command (v, w) for `dur` seconds, calling on_step() after each step."""
        t0 = self.t()
        while self.t() - t0 < dur - 1e-9:
            self.cmd(v, w)
            if not self.step():
                return False
            if on_step:
                on_step()
        return True

    # ------------------------------------------------------------ sequence
    def run_sequence(self):
        """forward 2 s, spin left 2 s, arc 3 s, reverse 1.5 s, stop 1 s."""
        plan = [("forward", 0.2, 0.0, 2.0), ("spin left", 0.0, 1.0, 2.0),
                ("arc", 0.15, 0.4, 3.0), ("reverse", -0.1, 0.0, 1.5),
                ("stop", 0.0, 0.0, 1.0)]
        steps_per_s = int(round(1000.0 / self.dt))
        for name, v, w, dur in plan:
            x, y, th = self.pose()
            print(f"[drive_test] {name}: v={v} w={w} for {dur}s  start pose="
                  f"({x:.3f}, {y:.3f}, {math.degrees(th):.1f} deg)")
            counter = [0]

            def report():
                counter[0] += 1
                if counter[0] % steps_per_s == 0:
                    px, py, pth = self.pose()
                    print(f"  t={self.t():.2f}s pose=({px:.3f}, {py:.3f}, "
                          f"{math.degrees(pth):.1f} deg)")
            if not self.hold(v, w, dur, report):
                return
        self.stop()
        x, y, th = self.pose()
        print(f"[drive_test] sequence done, final pose=({x:.3f}, {y:.3f}, "
              f"{math.degrees(th):.1f} deg)")

    # ----------------------------------------------------------- stability
    def run_stability(self, dur=10.0):
        """Motors at 0 for `dur` s: robot must not drift, jitter or tip."""
        self.stop()
        node = self.robot.getSelf()
        x0, y0, th0 = self.pose()
        z0 = node.getPosition()[2]
        path = os.path.join(LOG_DIR, "stability.csv")
        max_xy = max_tilt = max_dz = 0.0
        with open(path, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["t", "x", "y", "z", "roll", "pitch", "yaw"])
            t0 = self.t()
            while self.t() - t0 < dur:
                if not self.step():
                    return
                x, y, _ = self.pose()
                z = node.getPosition()[2]
                roll, pitch, yaw = self.imu.getRollPitchYaw()
                wr.writerow([round(self.t(), 3), round(x, 5), round(y, 5),
                             round(z, 5), round(roll, 5), round(pitch, 5),
                             round(yaw, 5)])
                max_xy = max(max_xy, math.hypot(x - x0, y - y0))
                max_tilt = max(max_tilt, abs(roll), abs(pitch))
                max_dz = max(max_dz, abs(z - z0))
        _, _, th1 = self.pose()
        print(f"[stability] {dur:.0f} s at rest: max xy drift={max_xy * 1000:.2f} mm, "
              f"max |dz|={max_dz * 1000:.2f} mm, max tilt="
              f"{math.degrees(max_tilt):.3f} deg, yaw drift="
              f"{math.degrees(abs(wrap_to_pi(th1 - th0))):.3f} deg")
        print(f"[drive_test] wrote {path}")

    # ---------------------------------------------------------- kinematics
    def run_kinematics(self, settle_s=1.0, run_s=4.0):
        """Commanded (v, w) vs measured, from per-step GPS/IMU differences.

        Per step: v = (dx cos th + dy sin th) / dt (signed, so reverse is
        negative), w = wrap(dth) / dt (no wrap-around error on long spins).
        Averaged over run_s after settle_s of acceleration.
        """
        path = os.path.join(LOG_DIR, "kinematics_test.csv")
        cases = [(0.2, 0.0), (0.0, 1.0), (0.2, 0.5), (-0.1, 0.0)]
        rows = []
        for v_cmd, w_cmd in cases:
            self.hold(v_cmd, w_cmd, settle_s)
            acc = {"ds": 0.0, "dth": 0.0, "t": 0.0}
            prev = [self.pose(), self.t()]

            def sample():
                (x0, y0, th0), t0 = prev
                x1, y1, th1 = self.pose()
                t1 = self.t()
                thm = th0 + wrap_to_pi(th1 - th0) / 2.0
                acc["ds"] += (x1 - x0) * math.cos(thm) + (y1 - y0) * math.sin(thm)
                acc["dth"] += wrap_to_pi(th1 - th0)
                acc["t"] += t1 - t0
                prev[0], prev[1] = (x1, y1, th1), t1
            self.hold(v_cmd, w_cmd, run_s, sample)
            v_meas = acc["ds"] / acc["t"]
            w_meas = acc["dth"] / acc["t"]
            wl, wr_ = inverse_kinematics(v_cmd, w_cmd)
            rows.append([v_cmd, w_cmd, round(wl, 3), round(wr_, 3),
                         round(v_meas, 4), round(w_meas, 4),
                         _pct(v_meas, v_cmd), _pct(w_meas, w_cmd)])
            print(f"[kinematics] cmd=({v_cmd:+.2f} m/s, {w_cmd:+.2f} rad/s) "
                  f"wheels=({wl:+.2f}, {wr_:+.2f}) rad/s  "
                  f"meas=({v_meas:+.4f} m/s, {w_meas:+.4f} rad/s)  "
                  f"err v={rows[-1][6]} w={rows[-1][7]}")
            self.stop()
            self.hold(0.0, 0.0, 1.0)
        with open(path, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["v_cmd", "w_cmd", "wl_cmd", "wr_cmd", "v_meas",
                         "w_meas", "v_err_pct", "w_err_pct"])
            wr.writerows(rows)
        print(f"[drive_test] wrote {path}")

    # ------------------------------------------------------------ odometry
    def run_odometry(self):
        """Line / square / arc trajectories; encoder odometry vs GPS+IMU.

        Odometry is re-initialised to the GPS+IMU pose at the start of each
        trajectory, so the logged difference is pure odometry drift.
        """
        path = os.path.join(LOG_DIR, "odometry.csv")
        trajectories = {
            "line": [(0.2, 0.0, 5.0)],
            "square": [(0.2, 0.0, 2.5), (0.0, math.pi / 4, 2.0)] * 4,
            "arc": [(0.15, 0.4, 6.0)],
        }
        with open(path, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["t", "traj", "x_gps", "y_gps", "th_gps",
                         "x_odom", "y_odom", "th_odom"])
            for name, segs in trajectories.items():
                self.odo.reset(*self.pose())
                self.odo.update(self.enc_l.getValue(), self.enc_r.getValue())

                def log():
                    self.odo.update(self.enc_l.getValue(), self.enc_r.getValue())
                    x, y, th = self.pose()
                    ox, oy, oth = self.odo.pose()
                    wr.writerow([round(self.t(), 3), name, round(x, 4),
                                 round(y, 4), round(th, 4), round(ox, 4),
                                 round(oy, 4), round(oth, 4)])
                for v, w, dur in segs:
                    self.hold(v, w, dur, log)
                self.stop()
                self.hold(0.0, 0.0, 0.5, log)
                x, y, th = self.pose()
                ox, oy, oth = self.odo.pose()
                print(f"[odometry] {name}: final GPS=({x:.3f}, {y:.3f}, "
                      f"{math.degrees(th):.1f} deg) odom=({ox:.3f}, {oy:.3f}, "
                      f"{math.degrees(oth):.1f} deg) pos err="
                      f"{math.hypot(x - ox, y - oy) * 100:.2f} cm, heading err="
                      f"{math.degrees(abs(wrap_to_pi(th - oth))):.2f} deg")
        print(f"[drive_test] wrote {path}")

    # ---------------------------------------------------------- worldcheck
    def run_worldcheck(self):
        """Let objects settle, then compare DEF positions with config.py."""
        self.hold(0.0, 0.0, 1.0)
        check_world_consistency(self.robot)
        z = self.robot.getSelf().getPosition()[2]
        print(f"[worldcheck] robot base z after settling = {z:.4f} m")

    # ------------------------------------------------------------ snapshot
    def run_snapshot(self):
        """Top-down + angled screenshots of the 3D view (needs rendering)."""
        vp = self.robot.getFromDef("VIEWPOINT")
        cam = self.robot.getDevice(config.CAMERA_NAME)
        cam.enable(self.dt)             # overlay shows the robot's view
        self.hold(0.0, 0.0, 0.5)
        for fname, (pos, ori) in (("world_topdown.png", config.VIEW_TOPDOWN),
                                  ("world_angle.png", config.VIEW_ANGLED)):
            vp.getField("position").setSFVec3f([float(v) for v in pos.split()])
            vp.getField("orientation").setSFRotation([float(v) for v in ori.split()])
            self.hold(0.0, 0.0, 0.5)
            out = os.path.join(EVIDENCE_DIR, fname)
            self.robot.exportImage(out, 95)
            self.step()
            print(f"[snapshot] wrote {out}")


    def run_capture(self):
        """Spin one turn; save frames every 30 deg and print detections."""
        import cv2
        import numpy as np
        from detector import detect_colors
        cam = self.robot.getDevice(config.CAMERA_NAME)
        cam.enable(self.dt)
        w, h = cam.getWidth(), cam.getHeight()
        out_dir = os.path.join(EVIDENCE_DIR, "frames")
        os.makedirs(out_dir, exist_ok=True)
        self.hold(0.0, 0.0, 0.2)
        yaw_acc, next_shot, prev = 0.0, 0.0, self.pose()[2]
        while yaw_acc < 2 * math.pi + 0.1:
            if yaw_acc >= next_shot:
                img = np.frombuffer(cam.getImage(), np.uint8).reshape(h, w, 4)
                bgr = img[:, :, :3].copy()
                deg = int(round(math.degrees(self.pose()[2])))
                path = os.path.join(out_dir, f"spin_{int(round(math.degrees(next_shot))):03d}.png")
                cv2.imwrite(path, bgr)
                hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
                dets = detect_colors(bgr)
                print(f"[capture] yaw={deg:+4d} deg -> {os.path.basename(path)} "
                      f"dets={dets}")
                # where are the most saturated pixels? (diagnostic)
                sat = hsv[:, :, 1] > 120
                if sat.any():
                    hs = hsv[:, :, 0][sat]
                    vs = hsv[:, :, 2][sat]
                    print(f"          saturated px={int(sat.sum())} hue "
                          f"[{hs.min()}..{hs.max()}] val [{vs.min()}..{vs.max()}]")
                next_shot += math.radians(30)
            self.hold(0.0, config.FIND_ROT_SPEED, self.dt / 1000.0)
            th = self.pose()[2]
            yaw_acc += abs(wrap_to_pi(th - prev))
            prev = th
        self.stop()


def _pct(meas, cmd):
    if abs(cmd) < 1e-9:
        return "n/a"
    return f"{(meas - cmd) / cmd * 100:+.2f}%"


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    os.makedirs(EVIDENCE_DIR, exist_ok=True)
    dt = DriveTest()
    mode = os.environ.get("DRIVE_TEST_MODE", "sequence")
    print(f"[drive_test] mode = {mode}")
    runner = getattr(dt, f"run_{mode}", None)
    if runner is None:
        print(f"[drive_test] unknown mode {mode!r}, running sequence")
        runner = dt.run_sequence
    runner()
    dt.stop()
    sys.stdout.flush()
    dt.hold(0.0, 0.0, 0.5)              # let Webots relay console output
    # quit only for scripted CLI runs (mode given explicitly); when the world
    # is opened from the GUI, keep the window open so the result can be seen
    if config.AUTO_QUIT and "DRIVE_TEST_MODE" in os.environ:
        dt.robot.simulationQuit(0)


if __name__ == "__main__":
    main()
