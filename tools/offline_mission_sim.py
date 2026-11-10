"""offline_mission_sim.py — kinematic end-to-end run of the pick-and-place mission.

!! This is NOT Webots and NOT physics. !!
It replaces the simulator with a simple kinematic world so that the REAL
mission code — decision_maker.DecisionMaker, docking.Docker,
pick_place.PickPlaceController, arm_controller.ArmController, gripper.Gripper,
the A* planner on the real warehouse grid and the waypoint follower — can be
exercised end to end, deterministically, on any laptop and in pytest.

What is modelled (deliberately simple, documented so nobody mistakes it for
a physics result):
    * base: unicycle integration of the commanded (v, w);
    * LiDAR: 360 rays in the plane z = LIDAR_MOUNT_Z, ray-cast against the
      static map rectangles (walls, shelves, crates, racks) and floor parcels;
    * cameras: a parcel is "detected" when its bearing is inside the camera
      FOV; the position estimator returns the true centre + a fixed error, so
      the LiDAR face refinement in docking has something to correct;
    * arm joints: rate-limited tracking of the commanded positions;
    * fingers: close at their max speed until they touch a parcel side
      (then they stop and report GRIPPER_GRIP_FORCE on each pad);
    * palm range-finder: ray from the palm face along the gripper axis;
    * latch: presence = palm-face point within CONNECTOR_DISTANCE_TOL of a
      parcel-face centre with opposed normals; lock() makes the parcel follow
      the gripper rigidly; unlock() drops it onto the support underneath
      (floor or rack platform).

Usage:
    python tools/offline_mission_sim.py                # P3 -> Zone B, prints transitions
    python tools/offline_mission_sim.py --zone A --package P1 --plot
"""

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "controllers", "intellibot_controller"))

import numpy as np  # noqa: E402

import config  # noqa: E402
import path_utils  # noqa: E402
import shelving  # noqa: E402
from decision_maker import DecisionMaker  # noqa: E402
from dynamic_map import DynamicObstacleMap  # noqa: E402
from kinematics import wrap_to_pi  # noqa: E402
from localization import LocalizationManager  # noqa: E402
from safety import SpeedGovernor  # noqa: E402
from pick_place import build_manipulator  # noqa: E402
from warehouse_grid import build_warehouse_grid  # noqa: E402


# ============================================================== world model
class Parcel:
    def __init__(self, def_name, pkg_id, x, y, z):
        self.def_name = def_name
        self.pkg_id = pkg_id
        self.pos = np.array([x, y, z], dtype=float)
        self.yaw = 0.0
        self.attached = False
        self.offset = None          # (along, lateral, dz) in the gripper frame


class KinematicWorld:
    def __init__(self, start=None, estimate_error=(0.06, -0.04)):
        x, y, th = start if start is not None else config.ROBOT_START
        self.pose = [float(x), float(y), float(th)]
        self.parcels = {}
        for def_name, px, py, _color in config.PACKAGES:
            pid = def_name.replace("PKG_", "")
            self.parcels[def_name] = Parcel(def_name, pid, px, py, config.PKG_SIZE / 2)
        self.rects = [(cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2)
                      for (cx, cy, sx, sy) in config.STATIC_OBSTACLES]
        self.n_rays = config.LIDAR_RAYS
        self.angles = np.array([-math.pi + i * 2 * math.pi / self.n_rays
                                for i in range(self.n_rays)])
        self.estimate_error = estimate_error
        self.t = 0.0
        self.obstacles = []         # MovingBox list (dynamic obstacles)

    # -------------------------------------------------- dynamic obstacles
    def obstacle_rects(self, t=None):
        t = self.t if t is None else t
        return [o.rect(t) for o in self.obstacles if o.active(t)]

    # ------------------------------------------------------------- LiDAR
    def _parcel_rects_in_lidar_plane(self):
        h = config.PKG_SIZE / 2
        out = []
        for p in self.parcels.values():
            if abs(p.pos[2] - config.LIDAR_MOUNT_Z) <= h and not p.attached:
                out.append((p.pos[0] - h, p.pos[0] + h, p.pos[1] - h, p.pos[1] + h))
        return out

    def scan(self):
        """Vectorised slab-method ray casting of all rays against all rects."""
        x, y, th = self.pose
        rects = np.array(self.rects + self._parcel_rects_in_lidar_plane()
                         + self.obstacle_rects())
        dx = np.cos(th + self.angles)[:, None]
        dy = np.sin(th + self.angles)[:, None]
        with np.errstate(divide="ignore", invalid="ignore"):
            tx1 = (rects[None, :, 0] - x) / dx
            tx2 = (rects[None, :, 1] - x) / dx
            ty1 = (rects[None, :, 2] - y) / dy
            ty2 = (rects[None, :, 3] - y) / dy
        # rays parallel to a slab: inside -> (-inf, inf), outside -> empty
        par_x = np.abs(dx) < 1e-12
        inside_x = (rects[None, :, 0] <= x) & (x <= rects[None, :, 1])
        tx_lo = np.where(par_x, np.where(inside_x, -np.inf, np.inf), np.minimum(tx1, tx2))
        tx_hi = np.where(par_x, np.where(inside_x, np.inf, -np.inf), np.maximum(tx1, tx2))
        par_y = np.abs(dy) < 1e-12
        inside_y = (rects[None, :, 2] <= y) & (y <= rects[None, :, 3])
        ty_lo = np.where(par_y, np.where(inside_y, -np.inf, np.inf), np.minimum(ty1, ty2))
        ty_hi = np.where(par_y, np.where(inside_y, np.inf, -np.inf), np.maximum(ty1, ty2))
        tmin = np.maximum(tx_lo, ty_lo)
        tmax = np.minimum(tx_hi, ty_hi)
        hit = (tmax >= np.maximum(tmin, 0.0)) & (tmin > 0)
        d = np.where(hit, tmin, np.inf).min(axis=1)
        d[(d < config.LIDAR_MIN_RANGE) | (d > config.LIDAR_MAX_RANGE)] = np.inf
        return d, self.angles

    # ----------------------------------------------------------- cameras
    def detections(self):
        class Det:
            pass
        x, y, th = self.pose
        dets = []
        for p in self.parcels.values():
            if p.attached or p.pos[2] > 0.2:
                continue
            bearing = wrap_to_pi(math.atan2(p.pos[1] - y, p.pos[0] - x) - th)
            for is_rear, center in ((False, 0.0), (True, math.pi)):
                if abs(wrap_to_pi(bearing - center)) < config.CAMERA_FOV / 2:
                    d = Det()
                    d.pkg_id, d.color = p.pkg_id, config.PACKAGE_COLORS[p.pkg_id]
                    d.is_rear = is_rear
                    d.true_xy = (p.pos[0], p.pos[1])
                    dets.append(d)
        return dets

    def estimator(self, det, pose, scan, angles):
        ex, ey = self.estimate_error
        tx, ty = det.true_xy
        px, py = tx + ex, ty + ey
        return px, py, math.atan2(py - pose[1], px - pose[0]) - pose[2], \
            math.hypot(px - pose[0], py - pose[1])

    # -------------------------------------------------------------- base
    def step_base(self, v, w, dt):
        th = wrap_to_pi(self.pose[2] + w * dt)
        self.pose = [self.pose[0] + v * math.cos(th) * dt,
                     self.pose[1] + v * math.sin(th) * dt, th]

    def support_height(self, x, y):
        """Top of whatever is under (x, y): rack platform or floor."""
        for slot in shelving.all_slots():
            sx, sy, _ = slot.center
            if (abs(x - sx) < config.DEST_RACK_DEPTH / 2 + 0.05
                    and abs(y - sy) < config.DEST_RACK_BAY_WIDTH / 2):
                below = [t for t in config.DEST_RACK_LEVEL_TOPS if t <= self._z_query + 1e-6]
                return max(below) if below else 0.0
        return 0.0


class MovingBox:
    """Axis-aligned box obstacle: present for t_on <= t < t_off, moving with
    (vx, vy) and bouncing between its start point and `travel` metres away."""

    def __init__(self, x, y, sx, sy, vx=0.0, vy=0.0, travel=0.0, t_on=0.0,
                 t_off=math.inf, name="box"):
        self.x0, self.y0, self.sx, self.sy = x, y, sx, sy
        self.vx, self.vy, self.travel = vx, vy, travel
        self.t_on, self.t_off, self.name = t_on, t_off, name

    def active(self, t):
        return self.t_on <= t < self.t_off

    def center(self, t):
        sp = math.hypot(self.vx, self.vy)
        if sp == 0.0 or self.travel <= 0.0:
            return self.x0, self.y0
        d = (sp * (t - self.t_on)) % (2 * self.travel)
        d = d if d <= self.travel else 2 * self.travel - d
        return self.x0 + self.vx / sp * d, self.y0 + self.vy / sp * d

    def rect(self, t):
        cx, cy = self.center(t)
        return (cx - self.sx / 2, cx + self.sx / 2, cy - self.sy / 2, cy + self.sy / 2)


# robot footprint used for collision / clearance accounting (body + bumpers,
# front extended to the stowed parcel): x in [-0.17, 0.19], y in [-0.10, 0.10]
_FOOT = [(x, y) for x in np.linspace(-0.17, 0.19, 10) for y in (-0.10, 0.10)] + \
        [(x, y) for x in (-0.17, 0.19) for y in np.linspace(-0.10, 0.10, 6)]


def footprint_clearance(pose, rects):
    """Min distance (m) from the robot footprint to any rect; 0 = contact.
    Also returns whether the contact is at the front half."""
    x, y, th = pose
    c, s_ = math.cos(th), math.sin(th)
    pts = np.array([(x + c * px - s_ * py, y + s_ * px + c * py) for px, py in _FOOT])
    if not rects:
        return math.inf, True
    R = np.asarray(rects)
    dx = np.maximum(np.maximum(R[None, :, 0] - pts[:, None, 0], 0.0),
                    pts[:, None, 0] - R[None, :, 1])
    dy = np.maximum(np.maximum(R[None, :, 2] - pts[:, None, 1], 0.0),
                    pts[:, None, 1] - R[None, :, 3])
    d = np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0))
    i = int(np.argmin(d.min(axis=1)))
    return float(d.min()), _FOOT[i][0] >= 0.0


# ======================================================== kinematic arm driver
class KinematicArmDriver:
    """Implements the arm_driver interface on top of KinematicWorld."""

    def __init__(self, world, q0=None, dt=0.016):
        from arm_kinematics import PlanarPitchArm
        self.w = world
        self.kin = PlanarPitchArm.from_config()
        self.q = np.array(q0 if q0 is not None
                          else self.kin.inverse(*config.ARM_STOW_TCP, 0.0), dtype=float)
        self.q_cmd = self.q.copy()
        self.fingers = np.zeros(2)
        self.finger_cmd = 0.0
        self.force = np.zeros(2)
        self.latched = False
        self.dt = dt
        self.available = True
        self.missing = []

    # ---- geometry
    def _gripper_world(self):
        """Palm-face point, gripper axis (unit 3-D) and lateral axis in the world."""
        fk = self.kin.forward(self.q)
        x, y, th = self.w.pose
        c, s = math.cos(th), math.sin(th)
        wx, wz = fk["W"]
        ax = math.cos(fk["phi"])
        az = math.sin(fk["phi"])
        palm_r = (wx + config.GRIPPER_WRIST_TO_PALM * ax, wz + config.GRIPPER_WRIST_TO_PALM * az)
        palm = np.array([x + palm_r[0] * c, y + palm_r[0] * s, palm_r[1]])
        axis = np.array([ax * c, ax * s, az])
        lateral = np.array([-s, c, 0.0])
        return palm, axis, lateral

    def _parcel_between_fingers(self):
        palm, axis, lat = self._gripper_world()
        for p in self.w.parcels.values():
            rel = p.pos - palm
            along = rel @ axis
            side = rel @ lat
            up = rel[2] - along * axis[2]
            if (0.0 < along < config.GRIPPER_FINGER_SIZE[0] + config.PKG_SIZE / 2
                    and abs(side) < 0.03 and abs(up) < 0.04):
                return p
        return None

    # ---- physics-free update, call once per control step
    def step(self, dt):
        vmax = np.array(config.ARM_JOINT_MAX_VEL)
        self.q += np.clip(self.q_cmd - self.q, -vmax * dt, vmax * dt)
        stop = config.GRIPPER_FINGER_TRAVEL
        p = self._parcel_between_fingers()
        if p is not None and self.finger_cmd > 0:
            stop = config.GRIPPER_OPEN_HALF_WIDTH - config.PKG_SIZE / 2
        target = min(self.finger_cmd, stop)
        step = config.GRIPPER_FINGER_MAX_VEL * dt
        self.fingers += np.clip(target - self.fingers, -step, step)
        touching = p is not None and self.finger_cmd > stop - 1e-9 and \
            np.all(self.fingers >= stop - 1e-4)
        self.force[:] = config.GRIPPER_GRIP_FORCE if touching else 0.0
        # carried parcel follows the gripper
        palm, axis, lat = self._gripper_world()
        for p in self.w.parcels.values():
            if p.attached:
                a, l, d = p.offset
                p.pos = palm + axis * a + lat * l + np.array([0, 0, d])

    # ---- driver interface
    def read_joints(self):
        return list(self.q)

    def command_joints(self, q):
        self.q_cmd = np.array(self.kin.clamp(q), dtype=float)

    def read_fingers(self):
        return float(self.fingers[0]), float(self.fingers[1])

    def command_fingers(self, pos, force=None):
        self.finger_cmd = float(pos)

    def read_finger_forces(self):
        return float(self.force[0]), float(self.force[1])

    def read_palm_range(self):
        palm, axis, lat = self._gripper_world()
        best = config.GRIPPER_PALM_RANGE_MAX
        h = config.PKG_SIZE / 2
        for p in self.w.parcels.values():
            rel = p.pos - palm
            along = rel @ axis
            if along <= 0:
                continue
            perp = rel - axis * along
            if np.all(np.abs(perp) <= h + 1e-9):
                best = min(best, max(0.0, along - h))
        return best

    def latch_presence(self):
        palm, axis, _lat = self._gripper_world()
        h = config.PKG_SIZE / 2
        for p in self.w.parcels.values():
            if p.attached:
                return 1 if self.latched else 0
            for n in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)):
                n = np.array(n, dtype=float)
                face = p.pos + n * h
                if (np.linalg.norm(face - palm) < config.CONNECTOR_DISTANCE_TOL
                        and math.acos(max(-1, min(1, -(n @ axis)))) < config.CONNECTOR_AXIS_TOL):
                    return 1
        return 0

    def latch_lock(self):
        if self.latch_presence() != 1:
            self.latched = True       # latch state only, no physical link
            return
        self.latched = True
        palm, axis, lat = self._gripper_world()
        h = config.PKG_SIZE / 2
        for p in self.w.parcels.values():
            rel = p.pos - palm
            if np.linalg.norm(rel) < h + config.CONNECTOR_DISTANCE_TOL + 0.01:
                p.attached = True
                p.offset = (rel @ axis, rel @ lat, rel[2] - (rel @ axis) * axis[2])
                return

    def latch_unlock(self):
        self.latched = False
        for p in self.w.parcels.values():
            if p.attached:
                p.attached = False
                self.w._z_query = p.pos[2] - config.PKG_SIZE / 2
                p.pos[2] = self.w.support_height(p.pos[0], p.pos[1]) + config.PKG_SIZE / 2

    def latch_is_locked(self):
        return self.latched


# ================================================================ runner
def run_mission(package="P3", zone="B", start=None, max_time=900.0, dt=0.032,
                estimate_error=(0.06, -0.04), occupied_slots=(), quiet=True,
                record=False, csv_path=None, navigation="full", localization="ekf",
                governor=True, obstacles=(), displace=None, gps_noise=0.0,
                imu_noise=0.0, odom_scale_error=0.0, gps_dropout=None, seed=0):
    """Run the real FSM against the kinematic world. Returns a result dict.

    navigation   : "full"   = continuous LiDAR dynamic layer + replanning
                              triggers + safe-start planner + recovery
                   "legacy" = old behaviour (replan only on the follower's
                              safety stop, single-ray obstacle mark, A*
                              start snapping) — ablation baseline
    localization : "truth" (pose = ground truth, no estimator) or a
                   LocalizationManager mode "ekf" | "odom" | "gps", fed with
                   simulated sensors: encoder increments with a scale error
                   (odom_scale_error, e.g. 0.02 = 2 % slip), GPS with
                   gps_noise sigma (m) and an optional dropout window
                   gps_dropout=(t0, t1), IMU yaw with imu_noise sigma (rad).
    governor     : SpeedGovernor on (V_CRUISE, clearance-limited) or off
                   (legacy fixed V_MAX)
    obstacles    : MovingBox list added to the world (seen by the LiDAR)
    displace     : (t, dx, dy, dth) — the robot is moved instantly at time t
                   (collision / manual drag); the wheels do NOT see it.
    """
    lines = []
    rng = np.random.default_rng(seed)

    def log(msg):
        lines.append(msg)
        if not quiet:
            print(msg)

    old = (config.TASK_PACKAGE, config.TASK_PACKAGE_DEF, config.TASK_PACKAGE_COLOR,
           config.TASK_ZONE, config.INTERACTIVE_MODE)
    config.TASK_PACKAGE, config.TASK_PACKAGE_DEF = package, f"PKG_{package}"
    config.TASK_PACKAGE_COLOR = config.PACKAGE_COLORS[package]
    config.TASK_ZONE, config.INTERACTIVE_MODE = zone, False
    try:
        world = KinematicWorld(start, estimate_error)
        world.obstacles = list(obstacles)
        driver = KinematicArmDriver(world, dt=dt)
        manip, kin = build_manipulator(driver, log=log)
        grid = build_warehouse_grid()
        full = navigation == "full"

        def planner(s, g):
            return path_utils.plan_to(grid, s, g, resample_spacing=0.3,
                                      inflation_radius=config.INFLATION_RADIUS,
                                      safe_start_radius=config.SAFE_START_RADIUS if full else None)

        gov = SpeedGovernor(enabled=governor)
        fsm = DecisionMaker(planner, world.estimator, grid=grid, log=log,
                            manipulator=manip, kinematics=kin,
                            obstacle_map=DynamicObstacleMap(grid) if full else None,
                            governor=gov)
        loc = None if localization == "truth" else LocalizationManager(localization, log=log)
        for sid in occupied_slots:
            fsm.occupancy.mark(sid, "pre-occupied")
        states, traj = [fsm.state], []
        csv_log = None
        if csv_path:
            from logger import MANIP_COLUMNS, CsvLogger
            from pick_place import manip_log_row
            csv_log = CsvLogger(csv_path, MANIP_COLUMNS)
        t, k = 0.0, 0
        last_inc = (0.0, 0.0)
        pose_err = []
        collisions = {"front": 0, "rear": 0}
        in_contact = False
        min_clear = math.inf
        displaced = False
        durations = {}
        while t < max_time:
            world.t = t
            if displace is not None and not displaced and t >= displace[0]:
                displaced = True
                world.pose = [world.pose[0] + displace[1], world.pose[1] + displace[2],
                              wrap_to_pi(world.pose[2] + displace[3])]
                log(f"[SIM] [t={t:.2f}s] robot DISPLACED by {displace[1:]} (wheels did not see it)")
            scan, angles = world.scan()
            if loc is None:
                pose = tuple(world.pose)
            else:
                gps = None
                if not (gps_dropout and gps_dropout[0] <= t < gps_dropout[1]):
                    gps = (world.pose[0] + rng.normal(0, gps_noise) if gps_noise else world.pose[0],
                           world.pose[1] + rng.normal(0, gps_noise) if gps_noise else world.pose[1])
                yaw = world.pose[2] + (rng.normal(0, imu_noise) if imu_noise else 0.0)
                est = loc.step(t, last_inc, gps, wrap_to_pi(yaw))
                jump = loc.consume_pose_jump()
                if jump is not None:
                    fsm.notify_pose_jump(jump, t)
                pose = est if est is not None else tuple(world.pose)
                fsm.health["localization"] = loc.status
                fsm.health["loc_ok"] = loc.healthy()
                e = (pose[0] - world.pose[0], pose[1] - world.pose[1],
                     wrap_to_pi(pose[2] - world.pose[2]))
                P = loc.covariance()
                nees = float(np.array(e) @ np.linalg.solve(P, np.array(e))) if P is not None else math.nan
                pose_err.append((t, *e, nees))
            st_before = fsm.state
            v, w = fsm.update(pose, scan, angles, world.detections(), t, dt)
            durations[st_before] = durations.get(st_before, 0.0) + dt
            if states[-1] != fsm.state:
                states.append(fsm.state)
            if record:
                traj.append((t, *world.pose, fsm.state, *driver.q))
            if csv_log is not None and k % 3 == 0:
                csv_log.log(**manip_log_row(manip, t, fsm.state))
            k += 1
            if fsm.state in ("DONE", "ERROR"):
                break
            if fsm.state.startswith(("ARM_", "VERIFY_")):
                assert v == 0.0 and w == 0.0, "base moved during manipulation!"
            before = list(world.pose)
            world.step_base(v, w, dt)
            # contact accounting against walls/shelves/racks and dynamic boxes
            # (floor parcels are excluded: the pick approach is the docker's job)
            rects = world.rects + world.obstacle_rects(t + dt)
            clr, front = footprint_clearance(world.pose, rects)
            if fsm.state in ("NAVIGATE", "DELIVER", "RETURN_TO_CENTER"):
                min_clear = min(min_clear, clr)
            if clr <= 1e-6:
                world.pose = before                 # a contact stops the base
                if not in_contact:
                    collisions["front" if front else "rear"] += 1
                    log(f"[SIM] [t={t:.2f}s] CONTACT ({'front' if front else 'rear'}) "
                        f"in {fsm.state}")
                    fsm.notify_collision(front, t)
                in_contact = True
            else:
                in_contact = False
            dth = wrap_to_pi(world.pose[2] - before[2])
            ds = math.hypot(world.pose[0] - before[0], world.pose[1] - before[1])
            ds = math.copysign(ds, v) if v != 0 else 0.0
            last_inc = (ds * (1.0 + odom_scale_error), dth * (1.0 + odom_scale_error))
            driver.step(dt)
            t += dt
        if csv_log is not None:
            csv_log.close()
        parcel = world.parcels[f"PKG_{package}"]
        audit = None
        if fsm.target_slot is not None:
            audit = shelving.audit_placement(parcel.pos, fsm.target_slot)
        loc_stats = None
        if pose_err:
            e = np.array(pose_err)
            loc_stats = dict(rmse_xy=float(np.sqrt(np.mean(e[:, 1] ** 2 + e[:, 2] ** 2))),
                             max_xy=float(np.max(np.hypot(e[:, 1], e[:, 2]))),
                             rmse_theta=float(np.sqrt(np.mean(e[:, 3] ** 2))),
                             mean_nees=float(np.nanmean(e[:, 4])),
                             summary=loc.summary())
        return {"final_state": fsm.state, "states": states, "time": t,
                "report": fsm.mission_report, "parcel_pos": parcel.pos.tolist(),
                "slot": fsm.target_slot.slot_id if fsm.target_slot else None,
                "audit": audit, "log": lines, "fsm": fsm, "world": world,
                "trajectory": traj, "collisions": collisions,
                "min_clearance": min_clear, "replans": fsm.replans,
                "replan_reasons": dict(fsm.replan_reasons),
                "safety": gov.summary(), "localization": loc_stats,
                "pose_errors": pose_err,
                "state_durations": durations,
                "nav_time": sum(durations.get(k, 0.0) for k in
                                ("NAVIGATE", "DELIVER", "RETURN_TO_CENTER"))}
    finally:
        (config.TASK_PACKAGE, config.TASK_PACKAGE_DEF, config.TASK_PACKAGE_COLOR,
         config.TASK_ZONE, config.INTERACTIVE_MODE) = old


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--package", default="P3", choices=sorted(config.PACKAGE_COLORS))
    ap.add_argument("--zone", default="B", choices=sorted(config.DEST_RACKS))
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--csv", help="also write a manipulation log (logger.MANIP_COLUMNS)")
    args = ap.parse_args()
    res = run_mission(args.package, args.zone, quiet=args.quiet, csv_path=args.csv)
    print("\nOFFLINE KINEMATIC SIMULATION (not Webots, no physics)")
    print("states :", " -> ".join(res["states"]))
    print(f"result : {res['final_state']} after {res['time']:.1f} s simulated")
    if res["audit"] is not None:
        ok, err = res["audit"]
        print(f"parcel : {np.round(res['parcel_pos'], 3).tolist()} in slot {res['slot']}: "
              f"{ok} (xy {err['xy'] * 100:.1f} cm, z {err['z'] * 100:.1f} cm)")
    return 0 if res["final_state"] == "DONE" else 1


if __name__ == "__main__":
    sys.exit(main())
