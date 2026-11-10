"""manip_check.py — Webots validation controller for the arm, gripper and cameras.

World: worlds/warehouse_manip_check.wbt (identical to warehouse.wbt, only the
robot's controller differs). Mode via the MANIP_CHECK_MODE env var:

    cameras   front/rear camera validation. The robot is teleported to four
              headings; for each, BOTH frames are saved to docs/evidence/cameras/
              and the OpenCV detector result of each camera is compared with
              that camera's own Recognition node (Webots ground truth):
                * same package ids            -> colour channels correct
                * same image side (u) / row (v) -> no horizontal/vertical flip
                * a package is only reported by the camera that can see it
                                              -> front/rear never interchanged
              Report: docs/evidence/cameras/camera_validation.md (+ .json)
    joints    step each joint +-0.3 rad around the stow pose; tracking error,
              settle time and overshoot -> docs/evidence/manipulation/joint_check.md
    gripper   open / close with nothing in the gripper -> width, pad forces
    pick      robot teleported in front of P3 -> dock (LiDAR) -> pick -> verify
              attachment (lift + carry pose) -> put the parcel back on the floor
              -> verify release -> stow; Supervisor ground truth of the parcel
              is recorded at every step -> docs/evidence/manipulation/pick_check.md
    all       cameras, gripper, joints, pick (default)

Batch example (Git Bash / Linux):
    MANIP_CHECK_MODE=cameras webots --batch --mode=realtime --stdout --stderr \\
        worlds/warehouse_manip_check.wbt
Camera images need rendering: do NOT use --no-rendering / --minimize for
"cameras" or "pick".
"""

import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_HERE, "..", "intellibot_controller"))

from controller import Supervisor  # noqa: E402  (Webots API)


import config  # noqa: E402
import detector  # noqa: E402
import shelving  # noqa: E402
from arm_driver import ParcelAuditor, WebotsArmDriver  # noqa: E402
from camera_pipeline import FRONT, REAR  # noqa: E402
from docking import Docker  # noqa: E402
from kinematics import inverse_kinematics  # noqa: E402
from logger import MANIP_COLUMNS, CsvLogger  # noqa: E402
from perception import Perception  # noqa: E402
from pick_place import build_manipulator, manip_log_row  # noqa: E402

CAM_DIR = os.path.join(_ROOT, "docs", "evidence", "cameras")
MANIP_DIR = os.path.join(_ROOT, "docs", "evidence", "manipulation")


class ManipCheck:
    def __init__(self):
        self.robot = Supervisor()
        self.dt = int(self.robot.getBasicTimeStep())
        self.perc = Perception(self.robot, self.dt)
        self.driver = WebotsArmDriver(self.robot, self.dt)
        if not self.driver.available:
            print(f"[manip_check] arm devices missing: {self.driver.missing}")
        self.manip, self.kin = build_manipulator(self.driver)
        self.audit = ParcelAuditor(self.robot)
        self.node = self.robot.getSelf()
        for cam in (self.perc.front_cam, self.perc.rear_cam):
            if cam.device is not None:
                cam.device.recognitionEnable(self.dt)
        try:
            self.robot.getDevice(config.PEN_NAME).write(False)
        except Exception:
            pass
        self.log = CsvLogger(os.path.join(_ROOT, "logs", "manip_check.csv"), MANIP_COLUMNS)

    # ------------------------------------------------------------ helpers
    def t(self):
        return self.robot.getTime()

    def step(self, n=1):
        for _ in range(n):
            if self.robot.step(self.dt) == -1:
                raise SystemExit(0)

    def hold(self, seconds, v=0.0, w=0.0):
        wl, wr = inverse_kinematics(v, w)
        self.perc.set_wheel_speeds(wl, wr)
        self.step(max(1, int(seconds * 1000 / self.dt)))
        self.perc.set_wheel_speeds(0.0, 0.0)

    def teleport(self, x, y, yaw):
        self.node.getField("translation").setSFVec3f([x, y, 0.0])
        self.node.getField("rotation").setSFRotation([0, 0, 1, yaw])
        self.node.resetPhysics()
        self.hold(0.5)

    def run_manip(self, label, max_s=90.0):
        t0 = self.t()
        while self.t() - t0 < max_s:
            st = self.manip.update(self.t())
            self.log.log(**manip_log_row(self.manip, self.t(), label))
            if st in ("SUCCEEDED", "FAILED"):
                return st
            self.step()
        return "TIMEOUT"

    def parcel(self, def_name="PKG_P3"):
        p = self.audit.position(def_name)
        return None if p is None else [round(v, 4) for v in p]

    # ------------------------------------------------------------ cameras
    def recognized(self, cam):
        out = {}
        for o in cam.device.getRecognitionObjects():
            model = o.getModel()
            if model.startswith("PKG_"):
                u, v = o.getPositionOnImage()
                out[model.replace("PKG_", "")] = (float(u), float(v))
        return out

    def run_cameras(self):
        import cv2
        os.makedirs(CAM_DIR, exist_ok=True)
        # 1 m east of the parcels' x, between P2 and P3, four headings
        x, y = -2.0, -0.25
        poses = [("facing_west", math.pi), ("facing_east", 0.0),
                 ("facing_north", math.pi / 2), ("facing_south", -math.pi / 2)]
        rows, ok_all = [], True
        for name, yaw in poses:
            self.teleport(x, y, yaw)
            self.hold(0.3)
            frames = self.perc.get_camera_frames(self.t())
            for role, cam in ((FRONT, self.perc.front_cam), (REAR, self.perc.rear_cam)):
                fr = frames[role]
                if not fr.valid:
                    rows.append(dict(pose=name, camera=role, valid=False, verdict="FAIL",
                                     reason=fr.reason))
                    ok_all = False
                    continue
                dets = detector.detect_colors(fr.image, is_rear=(role == REAR))
                det = {d.pkg_id: d.centroid for d in dets}
                rec = self.recognized(cam)
                cv2.imwrite(os.path.join(CAM_DIR, f"{name}_{role}.png"), fr.image)
                cv2.imwrite(os.path.join(CAM_DIR, f"{name}_{role}_annotated.png"),
                            detector.annotate(fr.image, dets))
                problems = []
                for pid, (u, v) in rec.items():
                    if pid not in det:
                        problems.append(f"{pid} recognised but not detected")
                        continue
                    du, dv = det[pid][0] - u, det[pid][1] - v
                    if abs(du) > 15 or abs(dv) > 15:
                        problems.append(f"{pid} offset ({du:+.0f},{dv:+.0f}) px")
                    if (det[pid][0] - cam.width / 2) * (u - cam.width / 2) < 0 and abs(u - cam.width / 2) > 20:
                        problems.append(f"{pid} on the opposite image side (flip?)")
                for pid in det:
                    if pid not in rec:
                        problems.append(f"{pid} detected but not visible to this camera")
                verdict = "PASS" if not problems else "FAIL"
                ok_all &= verdict == "PASS"
                rows.append(dict(pose=name, camera=role, valid=True, health=fr.health,
                                 mean=round(fr.stats["mean"], 1), std=round(fr.stats["std"], 1),
                                 recognised=sorted(rec), detected=sorted(det),
                                 verdict=verdict, problems=problems))
                print(f"[cameras] {name:13s} {role:5s} health={fr.health:6s} "
                      f"recognised={sorted(rec)} detected={sorted(det)} -> {verdict} "
                      f"{'; '.join(problems)}")
        with open(os.path.join(CAM_DIR, "camera_validation.json"), "w") as fh:
            json.dump(rows, fh, indent=2)
        with open(os.path.join(CAM_DIR, "camera_validation.md"), "w") as fh:
            fh.write("# Camera validation (generated by controllers/manip_check, mode cameras)\n\n")
            fh.write(f"Robot at ({x}, {y}); FOV front {config.CAMERA_FOV} rad, rear "
                     f"{config.REAR_CAMERA_FOV} rad, planar. Ground truth = each camera's "
                     "own Recognition node.\n\n")
            fh.write("| pose | camera | health | mean/std | recognised | detected | verdict |\n")
            fh.write("|---|---|---|---|---|---|---|\n")
            for r in rows:
                fh.write(f"| {r['pose']} | {r['camera']} | {r.get('health', 'INVALID')} | "
                         f"{r.get('mean', '-')}/{r.get('std', '-')} | "
                         f"{', '.join(r.get('recognised', []))} | "
                         f"{', '.join(r.get('detected', []))} | {r['verdict']} "
                         f"{'; '.join(r.get('problems', []))} |\n")
            fh.write(f"\n**Overall: {'PASS' if ok_all else 'FAIL'}**\n")
        print(f"[cameras] overall {'PASS' if ok_all else 'FAIL'} -> {CAM_DIR}")
        return ok_all

    # ------------------------------------------------------------- joints
    def run_joints(self):
        os.makedirs(MANIP_DIR, exist_ok=True)
        self.teleport(0.0, -0.3, 0.0)
        self.manip.start_stow(self.t())
        self.run_manip("stow")
        q0 = self.manip.arm.joints()
        lines = ["| joint | step (rad) | final error (rad) | overshoot (rad) | settle (s) | result |",
                 "|---|---|---|---|---|---|"]
        ok_all = True
        for j, name in enumerate(("shoulder", "elbow", "wrist")):
            for d in (+0.3, -0.3):
                goal = list(q0)
                goal[j] += d
                if not self.kin.within_limits(goal):
                    continue
                ok, why = self.manip.arm.move_joints(goal, self.t(), name=f"{name}{d:+}")
                t0, peak = self.t(), 0.0
                st = "MOVING"
                while st == "MOVING" and self.t() - t0 < 15:
                    st = self.manip.arm.update(self.t())
                    q = self.manip.arm.joints()
                    if q is not None:
                        peak = max(peak, (q[j] - goal[j]) * math.copysign(1, d))
                    self.log.log(**manip_log_row(self.manip, self.t(), f"joint_{name}"))
                    self.step()
                q = self.manip.arm.joints()
                err = abs(q[j] - goal[j]) if q else float("nan")
                ok_j = st == "DONE" and err < config.ARM_JOINT_TOL
                ok_all &= ok_j
                lines.append(f"| {name} | {d:+.2f} | {err:.4f} | {max(peak, 0):.4f} | "
                             f"{self.t() - t0:.2f} | {'OK' if ok_j else 'FAIL ' + self.manip.arm.fault} |")
                self.manip.arm.move_joints(q0, self.t())
                while self.manip.arm.update(self.t()) == "MOVING":
                    self.step()
        with open(os.path.join(MANIP_DIR, "joint_check.md"), "w") as fh:
            fh.write("# Joint step check (manip_check, mode joints)\n\n" + "\n".join(lines) + "\n")
        print("\n".join(lines))
        return ok_all

    # ------------------------------------------------------------ gripper
    def run_gripper(self):
        self.teleport(0.0, -0.3, 0.0)
        g = self.manip.gripper
        res = {}
        for label, fn in (("open", g.open), ("close_empty", g.close), ("reopen", g.open)):
            fn(self.t())
            t0 = self.t()
            while g.update(self.t()) == "MOVING" and self.t() - t0 < 10:
                self.step()
            a = g.assess()
            res[label] = dict(status=g.status, width=round(a.width, 4),
                              forces=[round(a.force_left, 2), round(a.force_right, 2)],
                              palm=round(a.palm_range, 3), state=a.state)
            print(f"[gripper] {label}: {res[label]}")
        ok = (res["open"]["width"] > 2 * config.GRIPPER_OPEN_HALF_WIDTH - 0.01
              and res["close_empty"]["state"] in ("MISSED", "NO_CONTACT"))
        os.makedirs(MANIP_DIR, exist_ok=True)
        with open(os.path.join(MANIP_DIR, "gripper_check.json"), "w") as fh:
            json.dump(res, fh, indent=2)
        return ok

    # --------------------------------------------------------------- pick
    def run_pick(self):
        """Isolated pick -> lift/carry -> put back -> verify, with ground truth."""
        os.makedirs(MANIP_DIR, exist_ok=True)
        steps, ok = [], True
        px, py = next((x, y) for d, x, y, _c in config.PACKAGES if d == "PKG_P3")
        line = shelving.dock_line_for_parcel((px, py), math.pi)
        sx, sy, _ = shelving.predock_pose(line)
        self.teleport(sx + 0.05, sy + 0.02, math.pi + 0.05)
        steps.append(("start", self.parcel()))
        docker = Docker(None, parcel_estimate=(px + 0.04, py - 0.03),
                        preferred_heading=math.pi, name="DOCK_PICK")
        while docker.status == "RUNNING":
            scan, ang = self.perc.get_lidar()
            v, w = docker.update(self.perc.get_pose(), scan, ang, self.t())
            wl, wr = inverse_kinematics(v, w)
            self.perc.set_wheel_speeds(wl, wr)
            self.step()
        self.perc.set_wheel_speeds(0.0, 0.0)
        steps.append((f"dock {docker.status} {docker.reason}", self.parcel()))
        if docker.status != "DONE":
            return self._pick_report(steps, False)
        grasp = shelving.pick_tcp_for_parcel(docker.measured_range)
        for label, start in (
                ("pick", lambda: self.manip.start_pick(self.t(), grasp)),
                ("verify_attachment", lambda: self.manip.start_verify_attachment(self.t()))):
            start()
            st = self.run_manip(label)
            steps.append((f"{label} {st} {self.manip.reason}", self.parcel()))
            if st != "SUCCEEDED":
                return self._pick_report(steps, False)
        self.hold(3.0)
        steps.append(("carried 3 s", self.parcel()))
        self._snapshot("pick_check_carry.png")
        floor = {"x": grasp["x"], "pre_x": grasp["x"] - 0.06,
                 "z_hover": grasp["z"] + config.ARM_PLACE_HOVER,
                 "z_release": grasp["z"] + config.ARM_PLACE_RELEASE_DROP,
                 "retract_x": grasp["x"] - config.ARM_RETRACT_DIST}
        for label, start in (
                ("place_on_floor", lambda: self.manip.start_place(self.t(), floor)),
                ("verify_release", lambda: self.manip.start_verify_delivery(self.t(), floor))):
            start()
            st = self.run_manip(label)
            steps.append((f"{label} {st} {self.manip.reason}", self.parcel()))
            ok &= st == "SUCCEEDED"
            if st != "SUCCEEDED":
                break
        self.hold(1.0)
        steps.append(("1 s after release", self.parcel()))
        return self._pick_report(steps, ok)

    def _snapshot(self, name):
        try:
            self.robot.exportImage(os.path.join(MANIP_DIR, name), 90)
        except Exception as exc:
            print(f"[pick] snapshot failed: {exc}")

    def _pick_report(self, steps, ok):
        with open(os.path.join(MANIP_DIR, "pick_check.md"), "w") as fh:
            fh.write("# Isolated pick / carry / release check (manip_check, mode pick)\n\n")
            fh.write("Parcel position = Supervisor ground truth (evaluation only).\n\n")
            fh.write("| step | PKG_P3 world position (x, y, z) |\n|---|---|\n")
            for label, pos in steps:
                fh.write(f"| {label} | {pos} |\n")
            fh.write(f"\nManipulation checks: `{json.dumps(self.manip.last_check)}`\n")
            fh.write(f"\n**Overall: {'PASS' if ok else 'FAIL'}**\n")
        for label, pos in steps:
            print(f"[pick] {label:45s} parcel={pos}")
        print(f"[pick] overall {'PASS' if ok else 'FAIL'}")
        return ok


def main():
    chk = ManipCheck()
    mode = os.environ.get("MANIP_CHECK_MODE", "all")
    order = ["cameras", "gripper", "joints", "pick"] if mode == "all" else [mode]
    results = {}
    for m in order:
        print(f"[manip_check] ===== {m} =====")
        results[m] = getattr(chk, f"run_{m}")()
    chk.log.close()
    print(f"[manip_check] results: {results}")
    sys.stdout.flush()
    if "MANIP_CHECK_MODE" in os.environ:
        chk.robot.simulationQuit(0 if all(results.values()) else 1)


if __name__ == "__main__":
    main()
