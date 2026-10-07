"""decision_maker.py — the mission FSM (pure logic, no Webots).

States (project_summary.md §10.9):
    IDLE -> FIND_PACKAGE -> NAVIGATE -> PICK -> PLAN_DELIVERY -> DELIVER
         -> DONE, with ERROR as a terminal failure state.

Testability: the FSM receives injected callables, so unit tests run it with
fake planners/estimators and a simulated unicycle — no Webots required.

    planner(start_xy, goal_xy) -> (waypoints | None, info)
    estimator(detections, pose, scan, angles) -> (x, y, phi, d) | None

The LiDAR scan is passed straight through to the WaypointFollower, whose
BLOCKED status is the designated re-planning hook for the final eval.
"""

import math

import controller_output.intellibot_controller.config as config
from controller_output.intellibot_controller.waypoint_follower import WaypointFollower
from controller_output.intellibot_controller.logger import transition


class DecisionMaker:
    def __init__(self, planner, estimator, grid=None, log=transition):
        self.planner = planner          # callable -> (waypoints, info)
        self.estimator = estimator      # callable -> (x, y, phi, d) | None
        self.grid = grid                # OccupancyGrid for dynamic obstacles
        self.log = log

        self.state = "IDLE"
        self.t_start = 0.0
        self.pkg_pos = None             # estimated package world position
        self.approach_pose = None       # stand-off point we navigate to
        self.follower = None
        self.detection_streak = 0
        self.scan_angle_accum = 0.0
        self.pick_deadline = None
        self.replans = 0
        self.path_length_total = 0.0
        self.last_info = {}
        self.mission_report = None
        self._last_heading_scan = None

    # ------------------------------------------------------------------ api
    def update(self, pose, scan, angles, detections, sim_time, dt):
        """One control step. Returns (v, w) body command."""
        handler = getattr(self, f"_do_{self.state.lower()}")
        return handler(pose, scan, angles, detections, sim_time, dt)

    def _set_state(self, new, sim_time):
        old = self.state
        self.state = new
        self.log(f"[t={sim_time:.1f}s] STATE: {old} -> {new}")

    # ---------------------------------------------------------------- states
    def _do_idle(self, pose, scan, angles, detections, sim_time, dt):
        self.t_start = sim_time
        self.replans = 0
        self.path_length_total = 0.0
        self.detection_streak = 0
        self.scan_angle_accum = 0.0
        self.mission_report = None
        self.pkg_pos = None
        self.log(f"[t={sim_time:.1f}s] Task: pick {config.TASK_PACKAGE} "
                 f"({config.TASK_PACKAGE_COLOR}), deliver to Zone {config.TASK_ZONE}")
        self._set_state("FIND_PACKAGE", sim_time)
        return 0.0, 0.0

    def _do_find_package(self, pose, scan, angles, detections, sim_time, dt):
        # rotate in place, watching for the target color
        target = [d for d in detections if d.pkg_id == config.TASK_PACKAGE]
        if target:
            self.detection_streak += 1
        else:
            self.detection_streak = 0

        if self.detection_streak >= config.DETECTION_CONSEC:
            est = self.estimator(target[0], pose, scan, angles)
            if est is not None:
                px, py, phi, d = est
                self.pkg_pos = (px, py)
                if self.grid is not None:
                    self.grid.add_obstacle_world(px, py, config.PKG_SIZE / 2)
                self.log(f"[t={sim_time:.1f}s] Package {config.TASK_PACKAGE} "
                         f"detected at ({px:.2f}, {py:.2f}), range {d:.2f} m, "
                         f"bearing {math.degrees(phi):.1f} deg")
                self.scan_angle_accum = 0.0
                self._set_state("NAVIGATE", sim_time)
                return 0.0, 0.0

        self.scan_angle_accum += abs(config.FIND_ROT_SPEED) * dt
        if self.scan_angle_accum > config.FIND_MAX_TURNS * 2.0 * math.pi:
            self._set_state("ERROR", sim_time)
            self.log("ERROR: package never found during scan")
            return 0.0, 0.0
        return 0.0, config.FIND_ROT_SPEED

    def _do_navigate(self, pose, scan, angles, detections, sim_time, dt):
        if self.follower is None:
            # approach pose: on the robot->package line, APPROACH_DISTANCE away
            px, py = self.pkg_pos
            rx, ry, _ = pose
            dx, dy = rx - px, ry - py
            norm = math.hypot(dx, dy) or 1.0
            self.approach_pose = (px + dx / norm * config.APPROACH_DISTANCE,
                                  py + dy / norm * config.APPROACH_DISTANCE)
            wps, info = self.planner((rx, ry), self.approach_pose)
            self.last_info = info
            if wps is None:
                self._set_state("ERROR", sim_time)
                self.log("ERROR: no path to package approach pose")
                return 0.0, 0.0
            self.path_length_total += info.get("length_m", 0.0)
            self.follower = WaypointFollower(wps)
            self.log(f"[t={sim_time:.1f}s] A* path: {info.get('n_waypoints', '?')} "
                     f"waypoints, {info.get('length_m', 0):.2f} m, "
                     f"{info.get('runtime_ms', 0):.1f} ms, "
                     f"{info.get('expansions', '?')} expansions")

        v, w = self.follower.update(pose, scan, angles)
        if self.follower.status == "REACHED":
            self.follower = None
            self.pick_deadline = sim_time + config.PICK_PAUSE_S
            self._set_state("PICK", sim_time)
        elif self.follower.status == "BLOCKED":
            # Final eval: dynamic re-plan here. Mid eval: stop and report.
            self.log("[t={:.1f}s] BLOCKED during NAVIGATE (LiDAR safety stop)"
                     .format(sim_time))
            self.follower = None
            self._set_state("ERROR", sim_time)
            return 0.0, 0.0
        return v, w

    def _do_pick(self, pose, scan, angles, detections, sim_time, dt):
        if sim_time >= self.pick_deadline:
            px, py = self.pkg_pos
            d = math.hypot(pose[0] - px, pose[1] - py)
            self.log(f"[t={sim_time:.1f}s] Package {config.TASK_PACKAGE} — "
                     f"Distance: {d:.2f} m — PICK placeholder "
                     f"(attachment in final eval)")
            self._set_state("PLAN_DELIVERY", sim_time)
        return 0.0, 0.0

    def _do_plan_delivery(self, pose, scan, angles, detections, sim_time, dt):
        goal = config.ZONE_OF[config.TASK_ZONE]
        wps, info = self.planner((pose[0], pose[1]), goal)
        self.last_info = info
        if wps is None:
            self._set_state("ERROR", sim_time)
            self.log("ERROR: no path to delivery zone")
            return 0.0, 0.0
        self.path_length_total += info.get("length_m", 0.0)
        self.follower = WaypointFollower(wps)
        self.log(f"[t={sim_time:.1f}s] A* path: {info.get('n_waypoints', '?')} "
                 f"waypoints, {info.get('length_m', 0):.2f} m, "
                 f"{info.get('runtime_ms', 0):.1f} ms, "
                 f"{info.get('expansions', '?')} expansions")
        self._set_state("DELIVER", sim_time)
        return 0.0, 0.0

    def _do_deliver(self, pose, scan, angles, detections, sim_time, dt):
        v, w = self.follower.update(pose, scan, angles)
        if self.follower.status == "REACHED":
            self.follower = None
            self._set_state("DONE", sim_time)
            self._emit_report(sim_time)
        elif self.follower.status == "BLOCKED":
            self.log("[t={:.1f}s] BLOCKED during DELIVER (LiDAR safety stop)"
                     .format(sim_time))
            self.follower = None
            self._set_state("ERROR", sim_time)
            return 0.0, 0.0
        return v, w

    def _emit_report(self, sim_time):
        self.mission_report = {
            "time_s": sim_time - self.t_start,
            "path_length_m": self.path_length_total,
            "replans": self.replans,
            "collisions": "N/A (bumper in final eval)",
        }
        self.log(f"[t={sim_time:.1f}s] MISSION REPORT: "
                 f"time={self.mission_report['time_s']:.1f}s, "
                 f"path_length={self.mission_report['path_length_m']:.2f} m, "
                 f"replans={self.mission_report['replans']}, "
                 f"collisions={self.mission_report['collisions']}")

    def _do_done(self, pose, scan, angles, detections, sim_time, dt):
        if self.mission_report is None:
            self._emit_report(sim_time)
        return 0.0, 0.0

    def _do_error(self, pose, scan, angles, detections, sim_time, dt):
        return 0.0, 0.0
