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

import config  # noqa: E402
import path_utils  # noqa: E402
from decision_maker import DecisionMaker  # noqa: E402
from display_dash import Dashboard, set_state_led  # noqa: E402
from kinematics import inverse_kinematics  # noqa: E402
from logger import RunLogger  # noqa: E402
from mapping import front_min_range  # noqa: E402
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
    print(f"[INTELLIBOT] basic time step: {timestep} ms")
    print_env_versions()

    run_idx, log_suffix = apply_campaign_start(robot)
    perception = Perception(robot, timestep)
    check_world_consistency(robot)

    # P2 extras: dashboard + state LED (checklist 6.10 / 6.11)
    dash = None
    if config.DISPLAY_ENABLED:
        try:
            dash = Dashboard(robot.getDevice(config.DISPLAY_NAME),
                             build_warehouse_grid())
        except Exception as exc:                    # device missing -> carry on
            print(f"[INTELLIBOT] dashboard disabled: {exc}")
    led = None
    try:
        led = robot.getDevice(config.LED_NAME)
    except Exception:
        pass

    grid = build_warehouse_grid()
    planner = planner_factory(grid)

    def estimator(detection, pose, scan, angles):
        return perception.estimate_package_position(detection, pose,
                                                    scan, angles)

    fsm = DecisionMaker(planner, estimator, grid=grid)
    logger = RunLogger(os.path.join(_ROOT, "logs",
                                    f"run{log_suffix}.csv")).start()

    log_every = max(1, int(0.1 / (timestep / 1000.0)))  # ~10 Hz
    dash_every = max(1, int(0.5 / (timestep / 1000.0)))  # ~2 Hz dashboard
    step_i = 0
    while robot.step(timestep) != -1:
        sim_time = robot.getTime()
        pose = perception.get_pose()                    # x, y, theta (GPS+IMU)
        odom_pose, _ = perception.get_odometry()
        scan, angles = perception.get_lidar()
        frame = perception.get_frame()
        dets = perception.get_detections(frame)

        v, w = fsm.update(pose, scan, angles, dets, sim_time,
                          timestep / 1000.0)
        wl, wr = inverse_kinematics(v, w)
        perception.set_wheel_speeds(wl, wr)
        set_state_led(led, fsm.state)

        if dash is not None and step_i % dash_every == 0:
            wps = fsm.follower.waypoints if fsm.follower else None
            dash.draw(pose, fsm.state, sim_time, wps, fsm.pkg_pos)

        if step_i % log_every == 0:
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
        if fsm.state in ("DONE", "ERROR") and sim_time > fsm.t_start + 2.0:
            # mission over: stop cleanly (simulationQuit alone proved
            # unreliable in batch/realtime runs — run.csv kept growing to the
            # 300 s safety timeout after DONE)
            print(f"[INTELLIBOT] mission {fsm.state} — stopping controller")
            break
        if sim_time > 300.0:                            # hard safety timeout
            print("[INTELLIBOT] 300 s timeout — stopping")
            break

    # final row so the CSV records the terminal state even when DONE happens
    # between two 10 Hz log ticks
    pose = perception.get_pose()
    logger.log(t=round(robot.getTime(), 3),
               x=round(pose[0], 4), y=round(pose[1], 4), theta=round(pose[2], 4),
               state=fsm.state, wp_idx=-1, n_detections=0)
    logger.close()
    sys.stdout.flush()          # DONE/REPORT prints otherwise lost to buffering
    if config.AUTO_QUIT and fsm.state in ("DONE", "ERROR"):
        robot.simulationQuit(0 if fsm.state == "DONE" else 1)


if __name__ == "__main__":
    main()
