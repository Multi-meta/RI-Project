"""intellibot_controller.py — main entry point (Webots controller).

Wires everything together per the architecture data flow:
    sensors -> perception -> decision_maker (FSM) -> planner -> controller
            -> kinematics -> motors -> logger
"""

import math
import os
import sys

# Webots sets the controller folder as CWD for imports; keep module dir importable
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from controller import Supervisor  # noqa: E402  (Webots API)

import config  # noqa: E402
import path_utils  # noqa: E402
from decision_maker import DecisionMaker  # noqa: E402
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


def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    print(f"[INTELLIBOT] basic time step: {timestep} ms")

    perception = Perception(robot, timestep)
    check_world_consistency(robot)

    grid = build_warehouse_grid()
    planner = planner_factory(grid)

    def estimator(detection, pose, scan, angles):
        return perception.estimate_package_position(detection, pose,
                                                    scan, angles)

    fsm = DecisionMaker(planner, estimator, grid=grid)
    logger = RunLogger(os.path.join("logs", "run.csv")).start()

    log_every = max(1, int(0.1 / (timestep / 1000.0)))  # ~10 Hz
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
        if sim_time > 300.0:                            # hard safety timeout
            print("[INTELLIBOT] 300 s timeout — stopping")
            break

    logger.close()
    if config.AUTO_QUIT and fsm.state in ("DONE", "ERROR"):
        robot.simulationQuit(0 if fsm.state == "DONE" else 1)


if __name__ == "__main__":
    main()
