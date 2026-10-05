"""drive_test.py — minimal test controller (Webots).

Runs scripted motion sequences and logs data for Phase 3 validation:
    * basic sequence: forward, spin left, arc, reverse, stop (prints pose)
    * kinematics validation: commanded (v, w) vs measured (GPS/IMU finite
      differences) -> logs/kinematics_test.csv
    * odometry vs GPS: line / square / arc trajectories -> logs/odometry.csv

Mode is chosen by an env var DRIVE_TEST_MODE: "sequence" (default),
"kinematics", or "odometry".
"""

import math
import os
import sys

# reuse the shared pure modules from the main controller folder
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "intellibot_controller"))

from controller import Robot  # noqa: E402  (Webots API)

import config  # noqa: E402
from kinematics import inverse_kinematics, Odometry, forward_kinematics  # noqa: E402


class DriveTest:
    def __init__(self):
        self.robot = Robot()
        self.dt = int(self.robot.getBasicTimeStep())
        self.left = self.robot.getDevice(config.LEFT_MOTOR)
        self.right = self.robot.getDevice(config.RIGHT_MOTOR)
        for m in (self.left, self.right):
            m.setPosition(float("inf"))
        self.enc_l = self.robot.getDevice(config.LEFT_ENCODER)
        self.enc_r = self.robot.getDevice(config.RIGHT_ENCODER)
        self.enc_l.enable(self.dt)
        self.enc_r.enable(self.dt)
        self.gps = self.robot.getDevice(config.GPS_NAME)
        self.gps.enable(self.dt)
        self.imu = self.robot.getDevice(config.IMU_NAME)
        self.imu.enable(self.dt)
        self.odo = Odometry()
        self.kin_file = None
        self.odo_file = None

    def step(self):
        return self.robot.step(self.dt) != -1

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

    # ------------------------------------------------------------ sequences
    def run_sequence(self):
        """forward 2 s, spin left 2 s, arc 3 s, reverse 1.5 s, stop."""
        plan = [("forward", 0.2, 0.0, 2.0), ("spin left", 0.0, 1.0, 2.0),
                ("arc", 0.15, 0.4, 3.0), ("reverse", -0.1, 0.0, 1.5),
                ("stop", 0.0, 0.0, 1.0)]
        t0 = self.robot.getTime()
        for name, v, w, dur in plan:
            print(f"[drive_test] {name}: v={v} w={w} for {dur}s")
            while self.robot.getTime() - t0 < dur:
                if not self.step():
                    return
                self.cmd(v, w)
                if int((self.robot.getTime() - t0) / self.dt) % 32 == 0:
                    x, y, th = self.pose()
                    print(f"  t={self.robot.getTime():.1f}s pose=({x:.3f}, "
                          f"{y:.3f}, {math.degrees(th):.1f} deg)")
            t0 = self.robot.getTime()
        self.stop()
        print("[drive_test] sequence done")

    def run_kinematics(self):
        """Commanded (v, w) vs measured (GPS/IMU finite differences)."""
        import csv
        path = os.path.join("logs", "kinematics_test.csv")
        os.makedirs("logs", exist_ok=True)
        cases = [(0.2, 0.0), (0.0, 1.0), (0.2, 0.5), (-0.1, 0.0)]
        settle_s, run_s = 1.0, 4.0
        with open(path, "w", newline="") as f:
            wr_ = csv.writer(f)
            wr_.writerows(["t", "v_cmd", "w_cmd", "v_meas", "w_meas"])
            for v_cmd, w_cmd in cases:
                # settle
                t0 = self.robot.getTime()
                while self.robot.getTime() - t0 < settle_s:
                    self.step()
                    self.cmd(v_cmd, w_cmd)
                # measure
                x0, y0, th0 = self.pose()
                t_start = self.robot.getTime()
                while self.robot.getTime() - t_start < run_s:
                    self.step()
                    self.cmd(v_cmd, w_cmd)
                x1, y1, th1 = self.pose()
                dt = self.robot.getTime() - t_start
                dx, dy = x1 - x0, y1 - y0
                # for pure spin, w from heading delta; else chord geometry
                if abs(v_cmd) < 1e-6:
                    v_meas = 0.0
                    w_meas = (th1 - th0) / dt
                else:
                    chord = math.hypot(dx, dy)
                    dth = abs(th1 - th0)
                    if dth < 1e-4:
                        v_meas, w_meas = chord / dt, 0.0
                    else:
                        radius = chord / (2.0 * math.sin(dth / 2.0))
                        v_meas = radius * dth / dt
                        w_meas = (th1 - th0) / dt
                wr_.writerow([round(self.robot.getTime(), 3), v_cmd, w_cmd,
                              round(v_meas, 4), round(w_meas, 4)])
                print(f"[kinematics] cmd=({v_cmd}, {w_cmd}) "
                      f"meas=({v_meas:.3f}, {w_meas:.3f})")
                # pause between cases
                self.stop()
                t0 = self.robot.getTime()
                while self.robot.getTime() - t0 < 1.0:
                    self.step()
        print(f"[drive_test] wrote {path}")

    def run_odometry(self):
        """Line / square / arc trajectories; log odom vs GPS+IMU."""
        import csv
        path = os.path.join("logs", "odometry.csv")
        os.makedirs("logs", exist_ok=True)
        trajectories = {
            "line": [(0.2, 0.0, 5.0)],
            "square": [(0.2, 0.0, 2.5), (0.0, math.pi / 4, 2.0)] * 4,
            "arc": [(0.15, 0.4, 6.0)],
        }
        with open(path, "w", newline="") as f:
            wr_ = csv.writer(f)
            wr_.writerows(["t", "traj", "x_gps", "y_gps", "th_gps",
                           "x_odom", "y_odom", "th_odom"])
            for name, segs in trajectories.items():
                print(f"[drive_test] odometry trajectory: {name}")
                self.odo.reset()
                for v, w, dur in segs:
                    t0 = self.robot.getTime()
                    while self.robot.getTime() - t0 < dur:
                        self.step()
                        self.cmd(v, w)
                        x, y, th = self.pose()
                        self.odo.update(self.enc_l.getValue(),
                                        self.enc_r.getValue())
                        ox, oy, oth = self.odo.pose()
                        wr_.writerow([
                            round(self.robot.getTime(), 3), name,
                            round(x, 4), round(y, 4), round(th, 4),
                            round(ox, 4), round(oy, 4), round(oth, 4)])
                self.stop()
                t0 = self.robot.getTime()
                while self.robot.getTime() - t0 < 0.5:
                    self.step()
        print(f"[drive_test] wrote {path}")


def main():
    dt = DriveTest()
    mode = os.environ.get("DRIVE_TEST_MODE", "sequence")
    if mode == "kinematics":
        dt.run_kinematics()
    elif mode == "odometry":
        dt.run_odometry()
    else:
        dt.run_sequence()
    dt.stop()
    if config.AUTO_QUIT:
        dt.robot.simulationQuit(0)


if __name__ == "__main__":
    main()
