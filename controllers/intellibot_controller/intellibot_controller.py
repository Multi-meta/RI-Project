"""intellibot_controller.py — main entry point (Webots controller).

Wires everything together per the architecture data flow:
    sensors -> perception -> decision_maker (FSM) -> planner -> controller
            -> kinematics -> motors -> logger

Interactive-mode changes (config.INTERACTIVE_MODE = True):
  * The FSM starts in AWAIT_INPUT so the FIRST task prompt runs inside the
    simulation step loop via a background thread — Webots never times out.
  * After each delivery the FSM navigates back to the center, then goes to
    AWAIT_INPUT again — the simulation never auto-quits.
  * PackageCarrier tracks config.TASK_PACKAGE_DEF dynamically so it
    attaches the correct Solid node for each new pickup.
"""

import math
import os
import sys
import threading

# Webots sets the controller folder as CWD for imports; keep module dir importable
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, _HERE)

from controller import Supervisor  # noqa: E402  (Webots API)

import json  # noqa: E402

import numpy as np  # noqa: E402
import config  # noqa: E402
import path_utils  # noqa: E402
import shelving  # noqa: E402
from arm_driver import ParcelAuditor, WebotsArmDriver  # noqa: E402
from decision_maker import DecisionMaker  # noqa: E402
from pick_place import build_manipulator, manip_log_row  # noqa: E402
from display_dash import Dashboard, set_state_led  # noqa: E402
from dynamic_map import DynamicObstacleMap  # noqa: E402
from localization import LocalizationManager  # noqa: E402
from safety import SpeedGovernor  # noqa: E402
from kinematics import inverse_kinematics  # noqa: E402
from logger import MANIP_COLUMNS, CsvLogger, RunLogger  # noqa: E402
from mapping import front_min_range  # noqa: E402
from mission_kpis import MissionKPITracker  # noqa: E402
from perception import Perception, check_world_consistency  # noqa: E402
from warehouse_grid import build_warehouse_grid  # noqa: E402


def log_dir():
    """Where run logs go (INTELLIBOT_LOG_DIR overrides <repo>/logs)."""
    return os.environ.get("INTELLIBOT_LOG_DIR", os.path.join(_ROOT, "logs"))


def apply_env_overrides():
    """Batch/scenario runs: choose the task without the interactive dialog.

    INTELLIBOT_TASK_PACKAGE=P1|P2|P3, INTELLIBOT_TASK_ZONE=A|B,
    INTELLIBOT_INTERACTIVE=0|1 (tools/run_scenario.py sets these).

    Phase-18 ablations in Webots (tools/run_scenario.py A_* scenarios):
    INTELLIBOT_LOC_MODE=ekf|odom|gps, INTELLIBOT_GOVERNOR=0|1,
    INTELLIBOT_DYN_REPLAN=0|1.
    """
    pkg = os.environ.get("INTELLIBOT_TASK_PACKAGE")
    if pkg in config.PACKAGE_COLORS:
        config.TASK_PACKAGE = pkg
        config.TASK_PACKAGE_DEF = f"PKG_{pkg}"
        config.TASK_PACKAGE_COLOR = config.PACKAGE_COLORS[pkg]
    zone = os.environ.get("INTELLIBOT_TASK_ZONE")
    if zone in config.ZONE_OF:
        config.TASK_ZONE = zone
    inter = os.environ.get("INTELLIBOT_INTERACTIVE")
    if inter in ("0", "1"):
        config.INTERACTIVE_MODE = inter == "1"
    mode = os.environ.get("INTELLIBOT_LOC_MODE")
    if mode in ("ekf", "odom", "gps"):
        config.LOCALIZATION_MODE = mode
    gov = os.environ.get("INTELLIBOT_GOVERNOR")
    if gov in ("0", "1"):
        config.SPEED_GOVERNOR_ENABLED = gov == "1"
    dyn = os.environ.get("INTELLIBOT_DYN_REPLAN")
    if dyn in ("0", "1"):
        config.DYNAMIC_REPLANNING_ENABLED = dyn == "1"


def fast_sphere_vy(sim_time):
    """y velocity of the FAST_SPHERE obstacle at sim_time (m/s)."""
    speed = config.FAST_SPHERE_SPEED
    half = config.FAST_SPHERE_HALF_PERIOD
    return speed if int(sim_time / half) % 2 == 0 else -speed


def planner_factory(grid):
    """A* planner over the inflated known map (+ dynamic layer)."""
    def plan(start_xy, goal_xy):
        return path_utils.plan_to(grid, start_xy, goal_xy,
                                  resample_spacing=0.3,
                                  inflation_radius=config.INFLATION_RADIUS,
                                  safe_start_radius=config.SAFE_START_RADIUS)
    return plan


def apply_campaign_start(robot):
    """Repeatability campaign: RUN_INDEX=<i> env var selects start pose."""
    idx = int(os.environ.get("RUN_INDEX", "-1"))
    if idx < 0 or idx >= len(config.CAMPAIGN_STARTS):
        return None, ""
    x, y, yaw = config.CAMPAIGN_STARTS[idx]
    self_node = robot.getSelf()
    self_node.getField("translation").setSFVec3f([x, y, 0.0])
    self_node.getField("rotation").setSFRotation([0, 0, 1, yaw])
    self_node.resetPhysics()
    print(f"[CAMPAIGN] run {idx}: start ({x:.2f}, {y:.2f}, "
          f"{math.degrees(yaw):.0f} deg)")
    return idx, f"_{idx}"


class PackageCarrier:
    """LEGACY placeholder (worlds WITHOUT the arm only): teleports the task
    package Solid with the robot while fsm.carrying is set. It is NOT used
    when the manipulator is present — then the parcel is physically carried
    by the gripper latch (Webots Connector) and nothing is teleported.

    In interactive mode the target Solid changes every mission: reload_node()
    is called on every update() so it picks up config.TASK_PACKAGE_DEF changes.
    """
    CARRY_Z = (config.BODY_HEIGHT / 2 + 0.08) + config.PKG_SIZE / 2 + 0.03
    DROP_AHEAD = config.BODY_LENGTH / 2 + config.PKG_SIZE / 2 + 0.05

    def __init__(self, robot):
        self.robot = robot
        self.node = None
        self.attached = False
        self._last_def = None
        self.reload_node()

    def reload_node(self):
        """Look up the Solid node for the current config.TASK_PACKAGE_DEF."""
        def_name = config.TASK_PACKAGE_DEF
        if def_name == self._last_def:
            return
        self._last_def = def_name
        self.node = self.robot.getFromDef(def_name)
        if self.node is None:
            print(f"[INTELLIBOT] {def_name} not in world — package won't be carried")
        else:
            print(f"[INTELLIBOT] PackageCarrier now tracking {def_name}")
        self.attached = False

    def update(self, pose, carrying):
        self.reload_node()          # picks up DEF changes after AWAIT_INPUT
        if self.node is None:
            return
        x, y, theta = pose
        if carrying:
            self._place(x, y, self.CARRY_Z, theta)
            self.attached = True
        elif self.attached:
            self._place(x + self.DROP_AHEAD * math.cos(theta),
                        y + self.DROP_AHEAD * math.sin(theta),
                        config.PKG_SIZE / 2, theta)
            self.attached = False

    def _place(self, x, y, z, yaw):
        self.node.getField("translation").setSFVec3f([x, y, z])
        self.node.getField("rotation").setSFRotation([0, 0, 1, yaw])
        self.node.resetPhysics()


def print_env_versions():
    """Phase 0.1: prove numpy/cv2/matplotlib import inside the controller."""
    import numpy
    print(f"[INTELLIBOT] python {sys.version.split()[0]} | "
          f"numpy {numpy.__version__}")
    try:
        import cv2
        print(f"[INTELLIBOT] cv2 {cv2.__version__}")
    except ImportError:
        print("[INTELLIBOT] cv2 NOT importable — fix Webots Python command!")
    try:
        import matplotlib
        print(f"[INTELLIBOT] matplotlib {matplotlib.__version__}")
    except ImportError:
        pass


def _extended_set_led(led, state):
    """set_state_led wrapper that also handles the two new FSM states."""
    if led is None:
        return
    extra = {"RETURN_TO_CENTER": 0x00FFFF, "AWAIT_INPUT": 0xFF00FF}
    colour = extra.get(state)
    if colour is not None:
        try:
            led.set(colour)
        except Exception:
            pass
    else:
        set_state_led(led, state)


def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    robot_id = sys.argv[1] if len(sys.argv) > 1 else robot.getName()
    print(f"[{robot_id}] basic time step: {timestep} ms")
    print_env_versions()

    apply_env_overrides()
    run_idx, log_suffix = apply_campaign_start(robot)
    if robot_id != "INTELLIBOT":
        log_suffix = f"_{robot_id}" + log_suffix

    interactive = getattr(config, "INTERACTIVE_MODE", False)

    # NOTE: Do NOT block here with input() — Webots will force-kill the
    # controller if it doesn't call robot.step() within ~1 second.
    # In interactive mode the FSM starts in AWAIT_INPUT and the input prompt
    # runs in a background thread inside the step loop.

    perception = Perception(robot, timestep)
    check_world_consistency(robot)

    # P2 extras: dashboard + state LED
    dash = None
    if config.DISPLAY_ENABLED:
        try:
            dash = Dashboard(robot.getDevice(config.DISPLAY_NAME),
                             build_warehouse_grid())
        except Exception as exc:
            print(f"[{robot_id}] dashboard disabled: {exc}")
    led = None
    try:
        led = robot.getDevice(config.LED_NAME)
    except Exception:
        pass
    pen = None
    if config.PEN_ENABLED:
        try:
            pen = robot.getDevice(config.PEN_NAME)
            pen.write(True)
        except Exception:
            pass

    grid = build_warehouse_grid()
    planner = planner_factory(grid)

    def estimator(detection, pose, scan, angles):
        return perception.estimate_package_position(detection, pose,
                                                    scan, angles)

    # In interactive mode start directly in AWAIT_INPUT so the background
    # thread fires the first prompt without blocking the step loop.
    # Mobile manipulator (arm + gripper + latch). Falls back to the legacy
    # placeholder pickup only if the world has no arm devices.
    arm_driver = WebotsArmDriver(robot, timestep)
    manip, kin = None, None
    if arm_driver.available:
        manip, kin = build_manipulator(arm_driver)
        print(f"[{robot_id}] manipulator ready: shoulder/elbow/wrist + gripper + "
              f"latch '{config.GRIPPER_CONNECTOR_NAME}'")
    else:
        print(f"[{robot_id}] WARNING: no manipulator in this world "
              f"(missing {arm_driver.missing}) — PICK is a PLACEHOLDER")
    auditor = ParcelAuditor(robot)
    manip_log = (CsvLogger(os.path.join(log_dir(), f"manip{log_suffix}.csv"),
                           MANIP_COLUMNS) if manip is not None else None)

    obstacle_map = DynamicObstacleMap(grid) if config.DYNAMIC_REPLANNING_ENABLED else None
    governor = SpeedGovernor()
    fsm = DecisionMaker(planner, estimator, grid=grid, manipulator=manip,
                        kinematics=kin, obstacle_map=obstacle_map, governor=governor)
    print(f"[{robot_id}] navigation: localization={config.LOCALIZATION_MODE}, "
          f"dynamic replanning={'ON' if obstacle_map is not None else 'OFF'}, "
          f"speed governor={'ON (cruise %.2f m/s)' % config.V_CRUISE if governor.enabled else 'OFF (fixed %.2f m/s)' % config.V_MAX}")
    if interactive and run_idx is None:
        fsm.state = "AWAIT_INPUT"
        print("[INTELLIBOT] Interactive mode — waiting for task input in console...",
              flush=True)

    carrier = PackageCarrier(robot) if manip is None else None
    logger = RunLogger(os.path.join(log_dir(), f"run{log_suffix}.csv")).start()

    # Enumerate attached devices
    attached_devices = {}
    try:
        for i in range(robot.getNumberOfDevices()):
            dev = robot.getDeviceByIndex(i)
            attached_devices[dev.getName()] = dev
    except Exception:
        pass

    bumper_f = attached_devices.get("bumper_f")
    if bumper_f:
        bumper_f.enable(timestep)
    bumper_r = attached_devices.get("bumper_r")
    if bumper_r:
        bumper_r.enable(timestep)

    emitter = attached_devices.get("emitter")
    receiver = attached_devices.get("receiver")
    if receiver:
        receiver.enable(timestep)


    cart1_node, cart2_node = None, None
    fast_sphere_node, fast_cart_node = None, None
    try:
        cart1_node = robot.getFromDef("CART_1")
        cart2_node = robot.getFromDef("CART_2")
        fast_sphere_node = robot.getFromDef(getattr(config, "FAST_SPHERE_DEF", "FAST_SPHERE"))
        fast_cart_node = robot.getFromDef(getattr(config, "FAST_CART_DEF", "FAST_CART"))
    except Exception:
        pass

    # The pose the robot navigates with. Initialised from the first VALID
    # GPS + IMU sample inside the loop (sensors are NaN before the first
    # step); predicted from SIGNED wheel-encoder increments.
    loc = LocalizationManager()
    self_node = robot.getSelf()            # ground truth: EVALUATION ONLY
    gt_err = []                            # (ex, ey, eth, nees) for the report
    bumper_prev = {"f": False, "r": False}
    collisions = {"front": 0, "rear": 0}
    kpi_tracker = MissionKPITracker(robot_id=robot_id)

    log_every  = max(1, int(0.1 / (timestep / 1000.0)))   # ~10 Hz
    dash_every = max(1, int(0.5 / (timestep / 1000.0)))   # ~2 Hz
    step_i = 0
    finished = False
    # Only auto-quit in unattended campaign runs; never in interactive mode.
    scenario_run = os.environ.get("INTELLIBOT_INTERACTIVE") == "0"
    quit_at_end = (config.AUTO_QUIT and (run_idx is not None or scenario_run)
                   and not interactive)
    TERMINAL_STATES = ("DONE", "ERROR")

    def audit_and_save_report():
        """Evaluation-only ground-truth audit + JSON mission report."""
        report = dict(fsm.mission_report or {})
        if manip is not None and fsm.target_slot is not None:
            pos = auditor.position(config.TASK_PACKAGE_DEF)
            if pos is not None:
                ok, err = shelving.audit_placement(pos, fsm.target_slot)
                report["ground_truth_audit"] = {
                    "parcel_xyz": [round(v, 4) for v in pos],
                    "slot_center": [round(v, 4) for v in fsm.target_slot.center],
                    "in_slot": ok, "err_xy_m": round(err["xy"], 4),
                    "err_z_m": round(err["z"], 4),
                    "note": "Supervisor ground truth, evaluation only — not "
                            "used by the robot's decisions"}
                print(f"[AUDIT] {config.TASK_PACKAGE_DEF} at "
                      f"({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) vs slot "
                      f"{fsm.target_slot.slot_id} -> "
                      f"{'IN SLOT' if ok else 'NOT IN SLOT'} "
                      f"(xy err {err['xy'] * 100:.1f} cm, z err {err['z'] * 100:.1f} cm)")
                if ok != fsm.delivery_verified:
                    print("[AUDIT] WARNING: sensor verification and ground truth "
                          "disagree — investigate before claiming success")
        report.update(kpi_tracker.generate_report().get("summary", {}))
        report["localization"] = loc.summary()
        if gt_err:
            e = np.asarray(gt_err, dtype=float)
            report["localization_vs_ground_truth"] = {
                "samples": int(len(e)),
                "rmse_xy_m": round(float(np.sqrt(np.mean(e[:, 0] ** 2 + e[:, 1] ** 2))), 4),
                "rmse_theta_deg": round(float(np.degrees(np.sqrt(np.mean(e[:, 2] ** 2)))), 3),
                "max_xy_m": round(float(np.max(np.hypot(e[:, 0], e[:, 1]))), 4),
                "mean_nees": round(float(np.nanmean(e[:, 3])), 3),
                "note": "Supervisor ground truth, evaluation only"}
        report["collisions_bumper"] = dict(collisions)
        report["camera_health"] = {"final": dict(cam_health),
                                   "transitions": dict(cam_transitions)}
        report["safety"] = governor.summary()
        if obstacle_map is not None:
            report["dynamic_map"] = {"updates": obstacle_map.updates,
                                     "cells_marked_total": obstacle_map.marked_total}
        path = os.path.join(log_dir(), f"mission_report{log_suffix}.json")
        try:
            with open(path, "w") as fh:
                json.dump(report, fh, indent=2, default=str)
            print(f"[INTELLIBOT] mission report -> {path}")
        except OSError as exc:
            print(f"[INTELLIBOT] could not write mission report: {exc}")

    def finish():
        pose = loc.pose or perception.get_pose()
        logger.log(t=round(robot.getTime(), 3),
                   x=round(pose[0], 4), y=round(pose[1], 4),
                   theta=round(pose[2], 4),
                   state=fsm.state, wp_idx=-1, n_detections=0)
        logger.close()
        if manip_log is not None:
            manip_log.close()
        audit_and_save_report()
        sys.stdout.flush()

    cam_health = {}
    cam_transitions = {}

    while robot.step(timestep) != -1:
        sim_time = robot.getTime()
        pose = perception.get_pose()
        odom_pose, _ = perception.get_odometry()
        scan, angles = perception.get_lidar()
        frames = perception.get_camera_frames(sim_time)
        for role, fr in frames.items():          # log camera health transitions
            if cam_health.get(role) != fr.health:
                print(f"[CAMERA] [t={sim_time:.2f}s] {role}: "
                      f"{cam_health.get(role, '-')} -> {fr.health} {fr.reason}")
                cam_health[role] = fr.health
                cam_transitions[role] = cam_transitions.get(role, 0) + 1
        dets = perception.get_detections(frames["front"].image,
                                         frames["rear"].image)

        if cart1_node is not None:
            c1_vx = 0.4 if (int(sim_time / 12.5) % 2 == 0) else -0.4
            cart1_node.setVelocity([c1_vx, 0.0, 0.0, 0.0, 0.0, 0.0])
        if cart2_node is not None:
            c2_vy = 0.3 if (int(sim_time / 16.0) % 2 == 0) else -0.3
            cart2_node.setVelocity([0.0, c2_vy, 0.0, 0.0, 0.0, 0.0])

        # Drive the fast sphere across the main transit path: back and forth
        # along y, FAST_SPHERE_SPEED for FAST_SPHERE_HALF_PERIOD per direction
        # (speed x half period = the same 2.625 m sweep as before)
        if fast_sphere_node is not None:
            fast_sphere_node.setVelocity([0.0, fast_sphere_vy(sim_time), 0.0, 0.0, 0.0, 0.0])
        if fast_cart_node is not None:
            c_speed = getattr(config, "FAST_CART_SPEED", 0.70)
            c_vy = c_speed if (int(sim_time / 4.0) % 2 == 0) else -c_speed
            fast_cart_node.setVelocity([0.0, c_vy, 0.0, 0.0, 0.0, 0.0])

        # ---- bumpers: a collision EVENT on the rising edge (not every step)
        for key, dev, front in (("f", bumper_f, True), ("r", bumper_r, False)):
            hit = dev is not None and dev.getValue() > 0.0
            if hit and not bumper_prev[key]:
                kpi_tracker.record_collision(is_front=front)
                collisions["front" if front else "rear"] += 1
                print(f"[BUMPER] [t={sim_time:.2f}s] {'Front' if front else 'Rear'} "
                      f"bumper collision!")
                fsm.notify_collision(front, sim_time)
            bumper_prev[key] = hit
        fsm.health["collisions"] = dict(collisions)

        # ---- localisation (EKF): signed encoder increments + GPS + IMU
        gps_raw = perception.gps.getValues()
        imu_raw = perception.imu.getRollPitchYaw()
        gps_xy = (gps_raw[0], gps_raw[1]) if gps_raw is not None else None
        imu_yaw = imu_raw[2] if imu_raw is not None else None
        odo = perception.odometry
        pose_est = loc.step(sim_time, (odo.last_ds, odo.last_dtheta), gps_xy, imu_yaw)
        jump = loc.consume_pose_jump()
        if jump is not None:
            fsm.notify_pose_jump(jump, sim_time)
        operating_pose = pose_est if pose_est is not None else pose
        fsm.health["localization"] = loc.status
        fsm.health["loc_ok"] = loc.healthy()
        fsm.health["lidar"] = ("FAILED" if perception.lidar.getRangeImage() is None
                               else "OK")
        gt = None
        try:
            p_gt = self_node.getPosition()
            o_gt = self_node.getOrientation()
            gt = (p_gt[0], p_gt[1], math.atan2(o_gt[3], o_gt[0]))
        except Exception:
            gt = None
        if gt is not None and pose_est is not None and step_i % log_every == 0:
            ex, ey = pose_est[0] - gt[0], pose_est[1] - gt[1]
            eth = math.atan2(math.sin(pose_est[2] - gt[2]), math.cos(pose_est[2] - gt[2]))
            nees = math.nan
            P = loc.covariance()
            if P is not None:
                try:
                    e = np.array([ex, ey, eth])
                    nees = float(e @ np.linalg.solve(P, e))
                except np.linalg.LinAlgError:
                    pass
            gt_err.append((ex, ey, eth, nees))
            kpi_tracker.update_ekf_error(ex, ey, eth)

        if emitter and int(sim_time * 10) % 10 == 0:
            try:
                hb_msg = json.dumps({
                    'channel': 'heartbeat', 'sender_id': robot_id,
                    'message_type': 'heartbeat',
                    'payload': {'pose': [round(float(p), 4) for p in operating_pose],
                                'state': fsm.state, 'carrying': fsm.carrying},
                    'timestamp': round(sim_time, 3)
                })
                emitter.send(hb_msg.encode('utf-8'))
            except Exception:
                pass

        try:
            v, w = fsm.update(operating_pose, scan, angles, dets, sim_time,
                              timestep / 1000.0)
        except Exception as exc:
            # an exception used to kill the controller: the robot then sat
            # frozen (or kept rolling) with no message in the mission report
            import traceback
            traceback.print_exc()
            print(f"[INTELLIBOT] [t={sim_time:.2f}s] CONTROLLER EXCEPTION in state "
                  f"{fsm.state}: {exc} — base stopped, arm held, mission -> ERROR")
            v, w = 0.0, 0.0
            if manip is not None:
                manip.arm.hold()
            fsm.failure_reason = f"CONTROLLER_EXCEPTION in {fsm.state}: {exc}"
            fsm._enter_error(sim_time)
        wl, wr = inverse_kinematics(v, w)
        perception.set_wheel_speeds(wl, wr)
        if carrier is not None:
            carrier.update(operating_pose, fsm.carrying)
        if manip_log is not None and step_i % log_every == 0 and not finished:
            manip_log.log(**manip_log_row(manip, sim_time, fsm.state))
        _extended_set_led(led, fsm.state)

        if dash is not None and step_i % dash_every == 0:
            wps = fsm.follower.waypoints if fsm.follower else None
            ls = loc.summary()
            g = governor.last or {}
            occ = fsm.occupancy.summary()
            lines = [f"LOC {ls['mode']} {ls['status']} sig={ls['sigma_xy']} reloc={ls['relocalizations']}",
                     f"GPS {ls['gps']} IMU {ls['imu']} LIDAR {fsm.health.get('lidar')} "
                     f"CAM {'/'.join(cam_health.get(k, '-') for k in ('front', 'rear'))}",
                     f"replans {fsm.replans} recov {fsm.recoveries} dyn {obstacle_map.count() if obstacle_map else 0}",
                     f"v_cap {g.get('v_cap', 0):.2f} ({g.get('reason', '-')}) clr {g.get('clearance', float('nan')):.2f}",
                     f"bumps F{collisions['front']} R{collisions['rear']} carrying {int(fsm.carrying)}",
                     f"shelf {occ}"]
            dash.draw(operating_pose, fsm.state, sim_time, wps, fsm.pkg_pos,
                      extra={"dyn_points": obstacle_map.occupied_points() if obstacle_map else None,
                             "gt_pose": gt, "lines": lines,
                             "warn": loc.status not in ("OK",) or bool(g.get("emergency"))})

        if not finished and step_i % log_every == 0:
            logger.log(
                t=round(sim_time, 3),
                x=round(pose[0], 4), y=round(pose[1], 4),
                theta=round(pose[2], 4),
                x_odom=round(odom_pose[0], 4), y_odom=round(odom_pose[1], 4),
                theta_odom=round(odom_pose[2], 4),
                v_cmd=round(v, 4), w_cmd=round(w, 4),
                wl=round(wl, 3), wr=round(wr, 3),
                state=fsm.state,
                wp_idx=(fsm.follower.idx if fsm.follower else -1),
                cross_track_err=(round(fsm.follower.cross_track_err, 4)
                                 if fsm.follower else 0.0),
                lidar_front_min=round(front_min_range(scan, angles,
                                                      config.FRONT_CONE), 3),
                n_detections=len(dets),
                x_est=round(operating_pose[0], 4), y_est=round(operating_pose[1], 4),
                theta_est=round(operating_pose[2], 4),
                x_gt=round(gt[0], 4) if gt else "", y_gt=round(gt[1], 4) if gt else "",
                theta_gt=round(gt[2], 4) if gt else "",
                loc_status=loc.status, sigma_xy=loc.summary()["sigma_xy"],
                v_cap=(round(governor.last.get("v_cap", 0.0), 3)
                       if math.isfinite(governor.last.get("v_cap", math.inf)) else ""),
                cap_reason=governor.last.get("reason", ""),
                clearance=(round(governor.last.get("clearance", math.inf), 3)
                           if math.isfinite(governor.last.get("clearance", math.inf)) else ""),
                dyn_cells=obstacle_map.count() if obstacle_map else 0,
                replans=fsm.replans,
            )
        step_i += 1
        if finished:
            continue

        if fsm.state in TERMINAL_STATES and sim_time > fsm.t_start + 2.0:
            finished = True
            finish()
            if quit_at_end:
                print(f"[INTELLIBOT] mission {fsm.state} — stopping controller")
                break
            print(f"[INTELLIBOT] mission {fsm.state} — robot idle, "
                  f"dashboard stays live (pause/reset the simulation to stop)")
            sys.stdout.flush()
        elif sim_time > config.MISSION_TIMEOUT_S and not interactive:
            # (interactive sessions run until the operator ends them)
            print(f"[INTELLIBOT] {config.MISSION_TIMEOUT_S:.0f} s timeout — stopping")
            finished = True
            finish()
            break

    if not finished:
        finish()
    if pen is not None and fsm.state == "DONE":
        try:
            vp = robot.getFromDef("VIEWPOINT")
            pos, ori = config.VIEW_TOPDOWN
            vp.getField("position").setSFVec3f([float(v) for v in pos.split()])
            vp.getField("orientation").setSFRotation(
                [float(v) for v in ori.split()])
            for _ in range(12):
                robot.step(timestep)
            out = os.path.join(_ROOT, "docs", "evidence", "world_trail.png")
            robot.exportImage(out, 95)
            print(f"[INTELLIBOT] trail screenshot -> {out}")
        except Exception as exc:
            print(f"[INTELLIBOT] trail screenshot failed: {exc}")
    sys.stdout.flush()
    if quit_at_end and fsm.state in TERMINAL_STATES:
        robot.simulationQuit(0 if fsm.state == "DONE" else 1)


if __name__ == "__main__":
    main()
