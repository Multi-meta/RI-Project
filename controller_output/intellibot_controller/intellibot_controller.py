"""intellibot_controller.py — main entry point (Webots controller).

Wires everything together per the architecture data flow:
    sensors -> perception -> decision_maker (FSM) -> planner -> controller
            -> kinematics -> motors -> logger
"""

import math
import os
import sys

# Webots sets the controller folder as CWD for imports; keep module dir importable
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, _HERE)

from controller import Supervisor  # noqa: E402  (Webots API)

import numpy as np  # noqa: E402
import config  # noqa: E402
import path_utils  # noqa: E402
from decision_maker import DecisionMaker  # noqa: E402
from display_dash import Dashboard, set_state_led  # noqa: E402
from ekf_localization import EKFLocalizer  # noqa: E402
from kinematics import inverse_kinematics  # noqa: E402
from logger import RunLogger  # noqa: E402
from mapping import front_min_range  # noqa: E402
from mission_kpis import MissionKPITracker  # noqa: E402
from perception import Perception, check_world_consistency  # noqa: E402
from warehouse_grid import build_warehouse_grid  # noqa: E402


def planner_factory(grid):
    """A* planner over the inflated known map (+ dynamic layer)."""
    def plan(start_xy, goal_xy):
        return path_utils.plan_to(grid, start_xy, goal_xy,
                                  resample_spacing=0.3,
                                  inflation_radius=config.INFLATION_RADIUS)
    return plan


def apply_campaign_start(robot):
    """Repeatability campaign (checklist 6.8): RUN_INDEX=<i> env var selects
    start pose i from config.CAMPAIGN_STARTS; the supervisor teleports the
    robot there before the mission. Returns (index, log_suffix)."""
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
    """Moves the task package Solid with the robot while fsm.carrying is set
    (Supervisor teleport each step), then sets it down in front of the robot.

    The package rides above the body and the LiDAR plane, so it never shows
    up in the scan.
    """
    CARRY_Z = (config.BODY_HEIGHT / 2 + 0.08) + config.PKG_SIZE / 2 + 0.03
    DROP_AHEAD = config.BODY_LENGTH / 2 + config.PKG_SIZE / 2 + 0.05

    def __init__(self, robot):
        self.node = robot.getFromDef(config.TASK_PACKAGE_DEF)
        if self.node is None:
            print(f"[INTELLIBOT] {config.TASK_PACKAGE_DEF} not in world — "
                  f"package will not be carried")
        self.attached = False

    def update(self, pose, carrying):
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


def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    robot_id = sys.argv[1] if len(sys.argv) > 1 else robot.getName()
    print(f"[{robot_id}] basic time step: {timestep} ms")
    print_env_versions()

    run_idx, log_suffix = apply_campaign_start(robot)
    if robot_id != "INTELLIBOT":
        log_suffix = f"_{robot_id}" + log_suffix
    perception = Perception(robot, timestep)
    check_world_consistency(robot)

    # P2 extras: dashboard + state LED (checklist 6.10 / 6.11)
    dash = None
    if config.DISPLAY_ENABLED:
        try:
            dash = Dashboard(robot.getDevice(config.DISPLAY_NAME),
                             build_warehouse_grid())
        except Exception as exc:                    # device missing -> carry on
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
            pen.write(True)                     # draw the driven path (3.5)
        except Exception:
            pass

    grid = build_warehouse_grid()
    planner = planner_factory(grid)

    def estimator(detection, pose, scan, angles):
        return perception.estimate_package_position(detection, pose,
                                                    scan, angles)

    fsm = DecisionMaker(planner, estimator, grid=grid)
    carrier = PackageCarrier(robot)
    logger = RunLogger(os.path.join(_ROOT, "logs",
                                    f"run{log_suffix}.csv")).start()

    # Enumerate attached devices to prevent Webots warnings when querying missing devices
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

    # Phase 14 & 15 devices: Arm joints and Comms
    emitter = attached_devices.get("emitter")
    receiver = attached_devices.get("receiver")
    if receiver:
        receiver.enable(timestep)

    arm_devices = {}
    for dev_name in ["lift_motor", "arm_motor_1", "arm_motor_2", "wrist_motor",
                     "finger_motor_1", "finger_motor_2"]:
        if dev_name in attached_devices:
            arm_devices[dev_name] = attached_devices[dev_name]
    for dev_name in ["lift_sensor", "arm_sensor_1", "arm_sensor_2", "wrist_sensor",
                     "finger_sensor_1", "finger_sensor_2", "palm_ir"]:
        if dev_name in attached_devices:
            d = attached_devices[dev_name]
            d.enable(timestep)
            arm_devices[dev_name] = d

    cart1_node, cart2_node = None, None
    try:
        cart1_node = robot.getFromDef("CART_1")
        cart2_node = robot.getFromDef("CART_2")
    except Exception:
        pass

    init_pose = perception.get_pose()
    ekf = EKFLocalizer(
        x0=np.array([init_pose[0], init_pose[1], init_pose[2]], dtype=float),
        P0=np.diag([0.05**2, 0.05**2, (2*math.pi/180)**2]),
        Q=np.diag(config.EKF_Q)
    )
    init_odom_pose, _ = perception.get_odometry()
    last_odom_pose = init_odom_pose
    kpi_tracker = MissionKPITracker(robot_id=robot_id)

    log_every = max(1, int(0.1 / (timestep / 1000.0)))  # ~10 Hz
    dash_every = max(1, int(0.5 / (timestep / 1000.0)))  # ~2 Hz dashboard
    step_i = 0
    finished = False
    # quit at DONE only in unattended campaign runs; an interactive run keeps
    # stepping (wheels stopped) so the camera/dashboard overlays stay live
    quit_at_end = config.AUTO_QUIT and run_idx is not None

    def finish():
        # final row so the CSV records the terminal state even when DONE
        # happens between two 10 Hz log ticks
        pose = perception.get_pose()
        logger.log(t=round(robot.getTime(), 3),
                   x=round(pose[0], 4), y=round(pose[1], 4),
                   theta=round(pose[2], 4),
                   state=fsm.state, wp_idx=-1, n_detections=0)
        logger.close()
        sys.stdout.flush()      # DONE/REPORT prints otherwise lost to buffering

    while robot.step(timestep) != -1:
        sim_time = robot.getTime()
        pose = perception.get_pose()                    # x, y, theta (GPS+IMU)
        odom_pose, _ = perception.get_odometry()
        scan, angles = perception.get_lidar()
        frame = perception.get_frame()
        dets = perception.get_detections(frame)

        # Drive moving obstacles along lanes (ping-pong)
        if cart1_node is not None:
            c1_vx = 0.4 if (int(sim_time / 12.5) % 2 == 0) else -0.4
            cart1_node.setVelocity([c1_vx, 0.0, 0.0, 0.0, 0.0, 0.0])
        if cart2_node is not None:
            c2_vy = 0.3 if (int(sim_time / 16.0) % 2 == 0) else -0.3
            cart2_node.setVelocity([0.0, c2_vy, 0.0, 0.0, 0.0, 0.0])

        # Bumper collision detection
        if bumper_f is not None and bumper_f.getValue() > 0.0:
            kpi_tracker.record_collision(is_front=True)
            print(f"[BUMPER] [t={sim_time:.2f}s] Front bumper collision detected!")
        if bumper_r is not None and bumper_r.getValue() > 0.0:
            kpi_tracker.record_collision(is_front=False)
            print(f"[BUMPER] [t={sim_time:.2f}s] Rear bumper collision detected!")

        # EKF localization update
        ds = math.hypot(odom_pose[0] - last_odom_pose[0], odom_pose[1] - last_odom_pose[1])
        dtheta = odom_pose[2] - last_odom_pose[2]
        last_odom_pose = odom_pose
        if math.isfinite(ds) and math.isfinite(dtheta):
            ekf.predict(ds, dtheta)
        if math.isfinite(pose[0]) and math.isfinite(pose[1]):
            ekf.update_gps(np.array([pose[0], pose[1]]), np.diag(config.EKF_R_GPS), gate_threshold=config.EKF_GATE_GPS)
        if math.isfinite(pose[2]):
            ekf.update_imu(pose[2], config.EKF_R_IMU, gate_threshold=config.EKF_GATE_IMU)

        ekf_pose = ekf.get_pose()
        if math.isfinite(ekf_pose[0]) and math.isfinite(ekf_pose[1]) and math.isfinite(ekf_pose[2]):
            operating_pose = (float(ekf_pose[0]), float(ekf_pose[1]), float(ekf_pose[2]))
        else:
            operating_pose = pose

        # Comms: broadcast periodic heartbeat (Phase 15.3)
        if emitter and int(sim_time * 10) % 10 == 0:
            import json
            try:
                hb_msg = json.dumps({
                    'channel': 'heartbeat',
                    'sender_id': robot_id,
                    'message_type': 'heartbeat',
                    'payload': {
                        'pose': [round(float(p), 4) for p in operating_pose],
                        'state': fsm.state,
                        'carrying': fsm.carrying
                    },
                    'timestamp': round(sim_time, 3)
                })
                emitter.send(hb_msg.encode('utf-8'))
            except Exception:
                pass

        v, w = fsm.update(operating_pose, scan, angles, dets, sim_time,
                          timestep / 1000.0)
        wl, wr = inverse_kinematics(v, w)
        perception.set_wheel_speeds(wl, wr)
        carrier.update(operating_pose, fsm.carrying)
        set_state_led(led, fsm.state)

        if dash is not None and step_i % dash_every == 0:
            wps = fsm.follower.waypoints if fsm.follower else None
            dash.draw(pose, fsm.state, sim_time, wps, fsm.pkg_pos)

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
            )
        step_i += 1
        if finished:
            continue
        if fsm.state in ("DONE", "ERROR") and sim_time > fsm.t_start + 2.0:
            # mission over: stop logging (simulationQuit alone proved
            # unreliable in batch/realtime runs — run.csv kept growing to the
            # 300 s safety timeout after DONE)
            finished = True
            finish()
            if quit_at_end:
                print(f"[INTELLIBOT] mission {fsm.state} — stopping controller")
                break
            print(f"[INTELLIBOT] mission {fsm.state} — robot idle, "
                  f"dashboard stays live (pause/reset the simulation to stop)")
            sys.stdout.flush()
        elif sim_time > 300.0:                          # hard safety timeout
            print("[INTELLIBOT] 300 s timeout — stopping")
            finished = True
            finish()
            break

    if not finished:            # Webots closed / reverted mid-mission
        finish()
    # Phase 3.5 evidence: top-down screenshot with the orange pen trail
    if pen is not None and fsm.state == "DONE":
        try:
            vp = robot.getFromDef("VIEWPOINT")
            pos, ori = config.VIEW_TOPDOWN
            vp.getField("position").setSFVec3f([float(v) for v in pos.split()])
            vp.getField("orientation").setSFRotation(
                [float(v) for v in ori.split()])
            for _ in range(12):
                robot.step(timestep)            # let the camera render
            out = os.path.join(_ROOT, "docs", "evidence", "world_trail.png")
            robot.exportImage(out, 95)
            print(f"[INTELLIBOT] trail screenshot -> {out}")
        except Exception as exc:
            print(f"[INTELLIBOT] trail screenshot failed: {exc}")
    sys.stdout.flush()          # DONE/REPORT prints otherwise lost to buffering
    if quit_at_end and fsm.state in ("DONE", "ERROR"):
        robot.simulationQuit(0 if fsm.state == "DONE" else 1)


if __name__ == "__main__":
    main()
