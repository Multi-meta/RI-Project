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
    """Moves the task package Solid with the robot while fsm.carrying is set.

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
    fsm = DecisionMaker(planner, estimator, grid=grid)
    if interactive and run_idx is None:
        fsm.state = "AWAIT_INPUT"
        print("[INTELLIBOT] Interactive mode — waiting for task input in console...",
              flush=True)

    carrier = PackageCarrier(robot)
    logger = RunLogger(os.path.join(_ROOT, "logs",
                                    f"run{log_suffix}.csv")).start()

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
    fast_sphere_node, fast_cart_node = None, None
    try:
        cart1_node = robot.getFromDef("CART_1")
        cart2_node = robot.getFromDef("CART_2")
        fast_sphere_node = robot.getFromDef(getattr(config, "FAST_SPHERE_DEF", "FAST_SPHERE"))
        fast_cart_node = robot.getFromDef(getattr(config, "FAST_CART_DEF", "FAST_CART"))
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

    log_every  = max(1, int(0.1 / (timestep / 1000.0)))   # ~10 Hz
    dash_every = max(1, int(0.5 / (timestep / 1000.0)))   # ~2 Hz
    step_i = 0
    finished = False
    # Only auto-quit in unattended campaign runs; never in interactive mode.
    quit_at_end = config.AUTO_QUIT and run_idx is not None and not interactive
    TERMINAL_STATES = ("DONE", "ERROR")

    def finish():
        pose = perception.get_pose()
        logger.log(t=round(robot.getTime(), 3),
                   x=round(pose[0], 4), y=round(pose[1], 4),
                   theta=round(pose[2], 4),
                   state=fsm.state, wp_idx=-1, n_detections=0)
        logger.close()
        sys.stdout.flush()

    while robot.step(timestep) != -1:
        sim_time = robot.getTime()
        pose = perception.get_pose()
        odom_pose, _ = perception.get_odometry()
        scan, angles = perception.get_lidar()
        frame = perception.get_frame()
        dets = perception.get_detections(frame)

        if cart1_node is not None:
            c1_vx = 0.4 if (int(sim_time / 12.5) % 2 == 0) else -0.4
            cart1_node.setVelocity([c1_vx, 0.0, 0.0, 0.0, 0.0, 0.0])
        if cart2_node is not None:
            c2_vy = 0.3 if (int(sim_time / 16.0) % 2 == 0) else -0.3
            cart2_node.setVelocity([0.0, c2_vy, 0.0, 0.0, 0.0, 0.0])

        # Drive fast-moving obstacles (0.75 m/s sphere, 0.70 m/s cart) across main transit path
        if fast_sphere_node is not None:
            s_speed = getattr(config, "FAST_SPHERE_SPEED", 0.75)
            s_vy = s_speed if (int(sim_time / 3.5) % 2 == 0) else -s_speed
            fast_sphere_node.setVelocity([0.0, s_vy, 0.0, 0.0, 0.0, 0.0])
        if fast_cart_node is not None:
            c_speed = getattr(config, "FAST_CART_SPEED", 0.70)
            c_vy = c_speed if (int(sim_time / 4.0) % 2 == 0) else -c_speed
            fast_cart_node.setVelocity([0.0, c_vy, 0.0, 0.0, 0.0, 0.0])

        if bumper_f is not None and bumper_f.getValue() > 0.0:
            kpi_tracker.record_collision(is_front=True)
            print(f"[BUMPER] [t={sim_time:.2f}s] Front bumper collision!")
        if bumper_r is not None and bumper_r.getValue() > 0.0:
            kpi_tracker.record_collision(is_front=False)
            print(f"[BUMPER] [t={sim_time:.2f}s] Rear bumper collision!")

        ds = math.hypot(odom_pose[0] - last_odom_pose[0],
                        odom_pose[1] - last_odom_pose[1])
        dtheta = odom_pose[2] - last_odom_pose[2]
        last_odom_pose = odom_pose
        if math.isfinite(ds) and math.isfinite(dtheta):
            ekf.predict(ds, dtheta)
        if math.isfinite(pose[0]) and math.isfinite(pose[1]):
            ekf.update_gps(np.array([pose[0], pose[1]]),
                           np.diag(config.EKF_R_GPS),
                           gate_threshold=config.EKF_GATE_GPS)
        if math.isfinite(pose[2]):
            ekf.update_imu(pose[2], config.EKF_R_IMU,
                           gate_threshold=config.EKF_GATE_IMU)

        ekf_pose = ekf.get_pose()
        if (math.isfinite(ekf_pose[0]) and math.isfinite(ekf_pose[1])
                and math.isfinite(ekf_pose[2])):
            operating_pose = (float(ekf_pose[0]), float(ekf_pose[1]),
                              float(ekf_pose[2]))
        else:
            operating_pose = pose

        if emitter and int(sim_time * 10) % 10 == 0:
            import json
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

        v, w = fsm.update(operating_pose, scan, angles, dets, sim_time,
                          timestep / 1000.0)
        wl, wr = inverse_kinematics(v, w)
        perception.set_wheel_speeds(wl, wr)
        carrier.update(operating_pose, fsm.carrying)
        _extended_set_led(led, fsm.state)

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

        if fsm.state in TERMINAL_STATES and sim_time > fsm.t_start + 2.0:
            finished = True
            finish()
            if quit_at_end:
                print(f"[INTELLIBOT] mission {fsm.state} — stopping controller")
                break
            print(f"[INTELLIBOT] mission {fsm.state} — robot idle, "
                  f"dashboard stays live (pause/reset the simulation to stop)")
            sys.stdout.flush()
        elif sim_time > 300.0:
            print("[INTELLIBOT] 300 s timeout — stopping")
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
