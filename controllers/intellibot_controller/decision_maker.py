"""decision_maker.py — the mission FSM (pure logic, no Webots).

States (robot WITHOUT an arm — legacy placeholder pickup, kept for old worlds
and the original unit tests):
    IDLE -> FIND_PACKAGE -> NAVIGATE -> PICK -> PLAN_DELIVERY -> DELIVER
         -> RETURN_TO_CENTER -> AWAIT_INPUT -> IDLE  (continuous interactive loop)
         -> DONE (only when INTERACTIVE_MODE = False), ERROR as terminal failure.

States with the mobile manipulator (a PickPlaceController is injected):
    IDLE -> ARM_STOW -> FIND_PACKAGE -> NAVIGATE -> DOCK_PICK -> ARM_PICK
         -> VERIFY_ATTACHMENT -> PLAN_DELIVERY -> DELIVER -> DOCK_PLACE
         -> ARM_PLACE -> VERIFY_DELIVERY -> DONE / RETURN_TO_CENTER
    failures: ARM_RECOVER (arm back to stow, then retry or ERROR)
    * the base is commanded (0, 0) in every ARM_* / VERIFY_* state and the
      arm only moves while the base is stopped — they never move together;
    * carrying becomes True only after VERIFY_ATTACHMENT confirms the parcel
      with sensors, and False once the latch is released on the shelf;
    * DONE requires VERIFY_DELIVERY (palm range-finder sees the parcel in the
      slot after the gripper backed away, latch open, fingers open).

Testability: the FSM receives injected callables, so unit tests run it with
fake planners/estimators and a simulated unicycle — no Webots required.

    planner(start_xy, goal_xy) -> (waypoints | None, info)
    estimator(detections, pose, scan, angles) -> (x, y, phi, d) | None

The LiDAR scan is passed straight through to the WaypointFollower, whose
BLOCKED status is the designated re-planning hook for the final eval.

Package carrying is exposed as the ``carrying`` flag (True from PICK until
DONE); the controller moves the package Solid accordingly via the Supervisor.

Interactive mode (config.INTERACTIVE_MODE = True):
    After each delivery the robot returns to the start center, then blocks on
    console input asking which color to pick and which zone to drop it in.
    The simulation loop continues stepping during the blocking input thanks to
    a background thread — the robot simply waits in place.

Pathfinding fix:
    _validate_pkg_estimate() cross-checks the camera+LiDAR position estimate
    against the config ground-truth.  When the estimate lands inside a static
    obstacle (the LiDAR ray hit an obstacle before reaching the package) or is
    more than MAX_PKG_ESTIMATE_ERR meters from the known position, the config
    ground-truth is used instead.

    _safe_approach_pose() tries multiple standoff angles until it finds one
    that is not inside an inflated obstacle cell, preventing the approach waypoint
    from being placed inside or directly behind a static obstacle.
"""

import math
import threading

import numpy as np

import config
from waypoint_follower import WaypointFollower
from logger import transition
import shelving
from docking import Docker
from recovery_behaviors import RecoveryBehavior, RecoveryConfig, RecoveryState
from safety import corridor_clearance

# states in which the base follows a planned route (speed governor in NAV
# mode, dynamic replanning active)
NAV_STATES = ("NAVIGATE", "DELIVER", "RETURN_TO_CENTER")


# --------------------------------------------------------------------- helpers

def _known_pkg_world_pos(pkg_id):
    """Return the config ground-truth (x, y) for a package id, or None."""
    for (def_name, x, y, color) in config.PACKAGES:
        pid = def_name.replace("PKG_", "")   # e.g. "PKG_P3" -> "P3"
        if pid == pkg_id:
            return (x, y)
    return None


def _validate_pkg_estimate(px, py, pkg_id, grid, max_err=1.5):
    """Check estimated package position against known config ground-truth.

    If the estimate is more than `max_err` metres from the config position,
    OR if it falls inside a static obstacle cell, fall back to the config
    ground-truth.  Returns validated (px, py).
    """
    known = _known_pkg_world_pos(pkg_id)
    if known is None:
        return px, py
    kx, ky = known
    err = math.hypot(px - kx, py - ky)
    if err > max_err:
        return kx, ky
    # Also reject estimates that land inside a known static obstacle
    if grid is not None:
        row, col = grid.world_to_cell(px, py)
        if grid.in_bounds(row, col) and grid.static[row, col]:
            return kx, ky
    return px, py


def _safe_approach_pose(px, py, rx, ry, approach_dist, grid):
    """Compute standoff point for package pickup.

    The packages sit along the west wall (x = -3.1). Standoff is placed on
    the open aisle (East, +x) side directly facing the package:
    (px + approach_dist, py).
    """
    if px < -2.5:
        # Directly in front of the package on the open floor (facing west)
        return px + approach_dist, py

    dx, dy = rx - px, ry - py
    norm = math.hypot(dx, dy) or 1.0
    return px + dx / norm * approach_dist, py + dy / norm * approach_dist


# ======================================================================= FSM =

class DecisionMaker:
    def __init__(self, planner, estimator, grid=None, log=transition,
                 input_callback=None, manipulator=None, kinematics=None,
                 obstacle_map=None, governor=None):
        self.planner = planner          # callable -> (waypoints, info)
        # Mobile manipulator (pick_place.PickPlaceController) — None = legacy
        # placeholder pickup (no arm in the world).
        self.manip = manipulator
        self.kin = kinematics           # arm_kinematics.PlanarPitchArm (reach checks)
        self.occupancy = shelving.RackOccupancy()
        self.docker = None
        self.target_slot = None
        self.place_targets = None
        self.grasp_attempts = 0
        self.place_attempts = 0
        self.failure_reason = None
        self.delivery_verified = False
        self.dock_log = []              # per docking: name, range, lateral, heading
        self._after_recover = None
        self._recover_reason = None
        self.estimator = estimator      # callable -> (x, y, phi, d) | None
        self.grid = grid                # OccupancyGrid for dynamic obstacles
        self.log = log
        # Optional callback called in a background thread when user input is
        # needed.  Signature: input_callback() -> None, must set config.TASK_*
        # before returning.
        self.input_callback = input_callback

        self.state = "IDLE"
        self.t_start = 0.0
        self.pkg_pos = None             # validated package world position
        self._pkg_seen = {}             # pkg_id -> last seen world estimate
        self.approach_pose = None       # stand-off point we navigate to
        self.follower = None
        self.detection_streak = 0
        self.scan_angle_accum = 0.0
        self.pick_deadline = None
        self.carrying = False           # package attached to the robot
        self.replans = 0
        self.path_length_total = 0.0
        self.last_info = {}
        self.mission_report = None
        self._last_heading_scan = None

        # Interactive-mode input state
        self._awaiting_input = False
        self._input_ready = False

        # Dynamic navigation (dynamic_map.DynamicObstacleMap) and the speed
        # governor (safety.SpeedGovernor). None = legacy behaviour.
        self.obstacle_map = obstacle_map
        self.governor = governor
        self.health = {}                # set by the controller each step
        self.nav_goal = None
        self.replan_events = []         # [{t, reason, from, ok, ...}]
        self.replan_reasons = {}
        self._last_replan_t = -1e9
        self._pending_jump = None
        self._pending_collision = None
        self._recovery = None
        self.recoveries = 0
        self._nav_recoveries = 0
        self._progress_ref = None
        self._last_cmd = (0.0, 0.0)
        self._urgent = False
        self._last_safety_stop = None
        self._blocked_start_time = None
        self._dyn_infl = None
        self._dyn_infl_stamp = -1
        self._dyn_yield_since = None
        self._dyn_clear_since = None
        self._yield_kind = None
        self._lane_override_until = -1e9
        self._lane_hist = []

    # ------------------------------------------------------------------ api
    def update(self, pose, scan, angles, detections, sim_time, dt):
        """One control step. Returns (v, w) body command (speed-governed)."""
        if self.state in NAV_STATES and self.obstacle_map is not None:
            keep_out = []
            if self.state == "NAVIGATE" and self.pkg_pos is not None:
                # the target parcel is approached on purpose — not an obstacle
                keep_out.append((self.pkg_pos[0], self.pkg_pos[1],
                                 config.PKG_SIZE * 0.75 + 0.05))
            self.obstacle_map.update(pose, scan, angles, sim_time, keep_out=keep_out,
                                     trusted=self.health.get("loc_ok", True))
        handler = getattr(self, f"_do_{self.state.lower()}")
        self._urgent = False
        v, w = handler(pose, scan, angles, detections, sim_time, dt)
        if self.state.startswith(("ARM_", "VERIFY_")):
            # the arm moves only with the base STOPPED: exact zero, no
            # deceleration ramp carried over from docking
            if self.governor is not None:
                self.governor.reset()
            self._last_cmd = (0.0, 0.0)
            return 0.0, 0.0
        if self.governor is not None:
            goal_dist = None
            if self.state in NAV_STATES and self.follower is not None:
                goal_dist = self.follower.remaining_length(pose)
            mode = "NAV" if self.state in NAV_STATES else "DOCK"
            v, w = self.governor.limit(v, w, dt, scan, angles, mode=mode,
                                       health=self.health, carrying=self.carrying,
                                       goal_dist=goal_dist, urgent=self._urgent)
        self._last_cmd = (v, w)
        return v, w

    # ------------------------------------------------- events from the robot
    def notify_pose_jump(self, jump, sim_time):
        """Localisation re-initialised (robot displaced): replan from the new
        pose at the next navigation step."""
        self._pending_jump = (jump, sim_time)

    def notify_collision(self, front, sim_time):
        """Bumper contact: stop, recover, replan (navigation states only)."""
        if self.state in NAV_STATES:
            self._pending_collision = (front, sim_time)

    def _begin_goal(self):
        """New navigation goal: fresh recovery budget, no stale events."""
        self._escape = None
        self._last_safety_stop = None
        self._nav_recoveries = 0
        self._recovery = None
        self._pending_collision = None
        self._blocked_start_time = None
        self._dyn_yield_since = None

    def _new_follower(self, wps):
        v_max = config.V_CRUISE if self.governor is not None and self.governor.enabled else None
        self.follower = WaypointFollower(wps, v_max=v_max)
        self._progress_ref = None
        return self.follower

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
        self.carrying = False
        self._pkg_seen = {}
        self.docker = None
        self.target_slot = None
        self.place_targets = None
        self.grasp_attempts = 0
        self.place_attempts = 0
        self.failure_reason = None
        self.delivery_verified = False
        self.dock_log = []
        self.log(f"[t={sim_time:.1f}s] Task: pick {config.TASK_PACKAGE} "
                 f"({config.TASK_PACKAGE_COLOR}), deliver to Zone {config.TASK_ZONE}")
        if self.manip is not None:
            # make sure the arm is stowed (known pose, inside the footprint)
            # before the base moves at all
            self.manip.start_stow(sim_time)
            self._set_state("ARM_STOW", sim_time)
            return 0.0, 0.0
        self._set_state("FIND_PACKAGE", sim_time)
        return 0.0, 0.0

    def _do_arm_stow(self, pose, scan, angles, detections, sim_time, dt):
        st = self.manip.update(sim_time)
        if st == "SUCCEEDED":
            self._set_state("FIND_PACKAGE", sim_time)
        elif st == "FAILED":
            self._mission_failed(f"ARM_NOT_READY: {self.manip.reason}", sim_time,
                                 recover=False)
        return 0.0, 0.0

    def _do_find_package(self, pose, scan, angles, detections, sim_time, dt):
        # rotate in place, watching for the target color
        target = [d for d in detections if d.pkg_id == config.TASK_PACKAGE]
        if target:
            self.detection_streak += 1
        else:
            self.detection_streak = 0

        # remember where ANY package was seen: non-target packages are
        # physical obstacles the approach path must plan around
        for det in detections:
            if det.pkg_id and det.pkg_id not in self._pkg_seen:
                est = self.estimator(det, pose, scan, angles)
                if est is not None and math.isfinite(est[0]) and math.isfinite(est[1]):
                    self._pkg_seen[det.pkg_id] = (est[0], est[1])

        if self.detection_streak >= config.DETECTION_CONSEC:
            est = self.estimator(target[0], pose, scan, angles)
            if est is not None and math.isfinite(est[0]) and math.isfinite(est[1]):
                px, py, phi, d = est

                # --- PATHFINDING BUG FIX ---
                # LiDAR can report a range to the OBSTACLE (e.g. OBST_2 at
                # (-0.8, -0.9)) that lies between the robot and the package,
                # placing pkg_pos inside or near that obstacle instead of at the
                # real package location.  Cross-check with config ground-truth.
                px, py = _validate_pkg_estimate(
                    px, py, config.TASK_PACKAGE, self.grid,
                    max_err=getattr(config, "MAX_PKG_ESTIMATE_ERR", 1.5))

                self.pkg_pos = (px, py)
                if self.grid is not None:
                    # Map other non-target packages as obstacles, but NOT the target package
                    for pkg_id, (ox, oy) in self._pkg_seen.items():
                        if pkg_id != config.TASK_PACKAGE:
                            self.grid.add_obstacle_world(ox, oy, config.PKG_SIZE / 2)
                others = {k: v for k, v in self._pkg_seen.items()
                          if k != config.TASK_PACKAGE}
                if others:
                    self.log("[t={:.1f}s] Other packages mapped as obstacles: "
                             "{}".format(sim_time,
                                         {k: (round(v[0], 2), round(v[1], 2))
                                          for k, v in others.items()}))
                self.log(f"[t={sim_time:.1f}s] Package {config.TASK_PACKAGE} "
                         f"validated at ({px:.2f}, {py:.2f}), lidar range {d:.2f} m, "
                         f"bearing {math.degrees(phi):.1f} deg")
                self.scan_angle_accum = 0.0
                self._set_state("NAVIGATE", sim_time)
                return 0.0, 0.0

        if target:
            # 360 view: target already visible in front/rear camera! Do not rotate body
            return 0.0, 0.0

        self.scan_angle_accum += abs(config.FIND_ROT_SPEED) * dt
        if self.scan_angle_accum > config.FIND_MAX_TURNS * 2.0 * math.pi:
            self._set_state("ERROR", sim_time)
            self.log("ERROR: package never found during scan")
            return 0.0, 0.0
        return 0.0, config.FIND_ROT_SPEED

    def _do_navigate(self, pose, scan, angles, detections, sim_time, dt):
        if self.follower is None:
            px, py = self.pkg_pos
            rx, ry, _ = pose

            # --- PATHFINDING BUG FIX ---
            # The standard standoff along the robot->package line can land inside
            # an inflated obstacle cell.  _safe_approach_pose() tries alternate
            # angles until it finds a free cell.
            self.approach_pose = _safe_approach_pose(
                px, py, rx, ry, config.APPROACH_DISTANCE, self.grid)

            self.log(f"[t={sim_time:.1f}s] Package at ({px:.2f},{py:.2f}), "
                     f"approach pose ({self.approach_pose[0]:.2f},{self.approach_pose[1]:.2f})")

            self._begin_goal()
            wps, info = self.planner((rx, ry), self.approach_pose)
            self.last_info = info
            if wps is None:
                self._set_state("ERROR", sim_time)
                self.log("ERROR: no path to package approach pose")
                return 0.0, 0.0
            self.path_length_total += info.get("length_m", 0.0)
            self._new_follower(wps)
            self.log(f"[t={sim_time:.1f}s] A* path: {info.get('n_waypoints', '?')} "
                     f"waypoints, {info.get('length_m', 0):.2f} m, "
                     f"{info.get('runtime_ms', 0):.1f} ms, "
                     f"{info.get('expansions', '?')} expansions")

        rx, ry, _ = pose
        ax, ay = self.approach_pose
        dist_to_approach = math.hypot(rx - ax, ry - ay)

        # Proximity arrival check for pickup
        if dist_to_approach < 0.28:
            self.follower = None
            self._blocked_start_time = None
            return self._arrived_at_parcel(sim_time)

        handled = self._nav_monitor(pose, scan, angles, self.approach_pose, sim_time, dt)
        if handled is not None:
            return handled
        v, w = self.follower.update(pose, scan, angles)
        if self.follower.status == "REACHED":
            self.follower = None
            self._blocked_start_time = None
            return self._arrived_at_parcel(sim_time)
        elif self.follower.status == "BLOCKED":
            return self._handle_blocked(pose, scan, angles, self.approach_pose, sim_time)
        else:
            self._blocked_start_time = None
        return v, w

    # ======================================================= dynamic replanning
    #
    # Every navigation step (NAVIGATE / DELIVER / RETURN_TO_CENTER):
    #   1. the LiDAR is fused into the dynamic layer (update() above);
    #   2. _nav_monitor() checks the triggers below and, when one fires,
    #      replans with A* FROM THE CURRENT ESTIMATED POSE to the SAME final
    #      goal (new WaypointFollower = waypoint index and heading PID reset);
    #   3. the follower's own LiDAR stop (BLOCKED) goes through
    #      _handle_blocked(): yield -> replan -> recovery.
    # Triggers: verified obstacle in the route corridor ahead, deviation from
    # the route, no progress (stuck), bumper collision, localisation pose
    # jump (robot displaced). A cooldown stops replan thrashing (pose jumps
    # and collisions bypass it). If no path exists from the current pose,
    # the recovery FSM runs: WAIT -> ROTATE (only if the swept circle is
    # clear) -> BACK UP (only if the rear is verified clear) -> REPLAN,
    # bounded by NAV_MAX_RECOVERIES, then ERROR with the reason.

    def _nav_monitor(self, pose, scan, angles, goal, sim_time, dt):
        """Returns (v, w) when it handled this step, None to keep following."""
        if self.follower is None:
            return None
        if self._recovery is not None:
            return self._recovery_step(pose, scan, angles, goal, sim_time, dt)
        if self._pending_collision is not None:
            front, _t = self._pending_collision
            self._pending_collision = None
            self._urgent = True
            self.log(f"[t={sim_time:.1f}s] COLLISION ({'front' if front else 'rear'} "
                     f"bumper) — stopping, recovering, then replanning")
            self._count_reason("COLLISION")
            self._start_recovery(sim_time, "COLLISION")
            # first move a few cm AWAY from the contact (only if that side is
            # verified clear), then the normal recovery sequence
            self._escape = {"dir": -1.0 if front else 1.0, "start": (pose[0], pose[1]),
                            "t0": sim_time}
            return 0.0, 0.0
        if self._pending_jump is not None:
            (dx, dy, dth), _t = self._pending_jump
            self._pending_jump = None
            self.log(f"[t={sim_time:.1f}s] Robot displaced by ({dx:+.2f}, {dy:+.2f}) m "
                     f"— replanning from the CURRENT pose")
            return self._replan(pose, goal, "POSE_JUMP", sim_time, bypass_cooldown=True)
        if self.obstacle_map is None:
            return None
        cooled = sim_time - self._last_replan_t >= config.REPLAN_COOLDOWN
        blocked_at = self._route_blocked(pose)
        moving_block = (blocked_at is not None and self.obstacle_map.moving_near(
            blocked_at[0], blocked_at[1], 0.6, sim_time))
        lane = None
        if blocked_at is None and sim_time >= self._lane_override_until:
            # route free right now — but it crosses the lane of a MOVING
            # obstacle (a bouncing cart) that is close to / coming towards it?
            lane = self._lane_conflict(pose, sim_time)
        if moving_block or lane is not None:
            kind = "block" if moving_block else "lane"
            if self._dyn_yield_since is None or self._yield_kind != kind:
                self._dyn_yield_since, self._yield_kind = sim_time, kind
                where = blocked_at if moving_block else lane
                self.log(f"[t={sim_time:.1f}s] MOVING obstacle "
                         f"{'on the route' if moving_block else 'approaching the route crossing'}"
                         f" at ({where[0]:.2f}, {where[1]:.2f}) — braking and yielding")
            self._dyn_clear_since = None
            waited = sim_time - self._dyn_yield_since
            if moving_block:
                if waited < config.DYN_YIELD_TIME:
                    return 0.0, 0.0
                self.log(f"[t={sim_time:.1f}s] Moving obstacle still on the route after "
                         f"{config.DYN_YIELD_TIME:.0f} s — replanning around it")
                self._dyn_yield_since = None
                self.log(f"[t={sim_time:.1f}s] Current path BLOCKED at waypoint {self.follower.idx}")
                return self._replan(pose, goal, "DYNAMIC_BLOCKED", sim_time)
            if waited < config.DYN_LANE_MAX_WAIT:
                return 0.0, 0.0
            # never deadlock: commit to the crossing (the governor and the
            # follower's LiDAR stop still protect the robot)
            self.log(f"[t={sim_time:.1f}s] Waited {waited:.0f} s at the crossing — "
                     f"crossing now (lane checks paused {config.DYN_LANE_COMMIT:.0f} s)")
            self._lane_override_until = sim_time + config.DYN_LANE_COMMIT
            self._dyn_yield_since = None
        elif self._dyn_yield_since is not None:
            if self._yield_kind == "block":
                # hysteresis: the route must stay clear for DYN_CLEAR_HOLD
                if self._dyn_clear_since is None:
                    self._dyn_clear_since = sim_time
                if sim_time - self._dyn_clear_since < config.DYN_CLEAR_HOLD:
                    return 0.0, 0.0
            self.log(f"[t={sim_time:.1f}s] Moving obstacle clear "
                     f"({sim_time - self._dyn_yield_since:.1f} s) — resuming")
            self._dyn_yield_since = None
            self._dyn_clear_since = None
        if blocked_at is not None and cooled:
            self.log(f"[t={sim_time:.1f}s] OBSTACLE DETECTED on the route at "
                     f"({blocked_at[0]:.2f}, {blocked_at[1]:.2f}) "
                     f"{math.hypot(blocked_at[0] - pose[0], blocked_at[1] - pose[1]):.2f} m ahead")
            self.log(f"[t={sim_time:.1f}s] Current path BLOCKED at waypoint {self.follower.idx}")
            return self._replan(pose, goal, "PATH_BLOCKED", sim_time)
        if self.follower.cross_track_err > config.REPLAN_DEVIATION_MAX and cooled:
            self.log(f"[t={sim_time:.1f}s] Off the route by "
                     f"{self.follower.cross_track_err:.2f} m")
            return self._replan(pose, goal, "DEVIATION", sim_time)
        if self._stuck(pose, sim_time) and cooled:
            self.log(f"[t={sim_time:.1f}s] STUCK: no progress for {config.STUCK_TIME:.0f} s")
            self._count_reason("STUCK")
            self._start_recovery(sim_time, "STUCK")
            return 0.0, 0.0
        return None

    def _count_reason(self, reason):
        self.replan_reasons[reason] = self.replan_reasons.get(reason, 0) + 1

    def _stuck(self, pose, sim_time):
        moving = abs(self._last_cmd[0]) > 0.05
        if self._progress_ref is None or not moving:
            self._progress_ref = (sim_time, pose[0], pose[1])
            return False
        t0, x0, y0 = self._progress_ref
        if math.hypot(pose[0] - x0, pose[1] - y0) > config.STUCK_MIN_PROGRESS:
            self._progress_ref = (sim_time, pose[0], pose[1])
            return False
        if sim_time - t0 > config.STUCK_TIME:
            self._progress_ref = (sim_time, pose[0], pose[1])
            return True
        return False

    def _dynamic_inflated(self):
        """Dynamic layer grown by the robot radius (cached per map update)."""
        om = self.obstacle_map
        if self._dyn_infl is not None and self._dyn_infl_stamp == om.updates:
            return self._dyn_infl
        g = self.grid
        dyn = g.dynamic
        n = int(math.ceil(config.ROBOT_RADIUS / g.res))
        out = dyn.copy()
        for dr in range(-n, n + 1):
            for dc in range(-n, n + 1):
                if dr * dr + dc * dc > n * n or (dr == 0 and dc == 0):
                    continue
                sh = np.zeros_like(dyn)
                sh[max(-dr, 0):g.n_rows + min(-dr, 0), max(-dc, 0):g.n_cols + min(-dc, 0)] = \
                    dyn[max(dr, 0):g.n_rows + min(dr, 0), max(dc, 0):g.n_cols + min(dc, 0)]
                out |= sh
        self._dyn_infl, self._dyn_infl_stamp = out, om.updates
        return out

    def _route_points(self, pose, lookahead):
        fol = self.follower
        pts = [(pose[0], pose[1])] + list(fol.waypoints[min(fol.idx, len(fol.waypoints) - 1):])
        out, travelled = [], 0.0
        step = self.grid.res * 0.5
        for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
            seg = math.hypot(bx - ax, by - ay)
            n = max(1, int(math.ceil(seg / step)))
            for k in range(1, n + 1):
                f = k / n
                d = travelled + seg * f
                if d > lookahead:
                    return out
                out.append((ax + (bx - ax) * f, ay + (by - ay) * f, d))
            travelled += seg
        return out

    def _lane_conflict(self, pose, sim_time):
        """First route point inside the recently swept lane of a MOVING
        obstacle while that obstacle is close to it or approaching it."""
        om = self.obstacle_map
        if self.follower is None or not self.follower.waypoints:
            return None
        lane = (sim_time - om.last_cleared) <= config.DYN_LANE_WINDOW
        if not lane.any():
            self._lane_hist = []
            return None
        occ_r, occ_c = np.nonzero(om.evidence_occupied())
        if occ_r.size == 0:
            self._lane_hist = []
            return None
        ox = self.grid.x_min + (occ_c + 0.5) * self.grid.res
        oy = self.grid.y_min + (occ_r + 0.5) * self.grid.res
        cross = None
        n = int(math.ceil(config.ROBOT_RADIUS / self.grid.res))
        rr, rc = self.grid.world_to_cell(pose[0], pose[1])
        if lane[max(0, rr - n):rr + n + 1, max(0, rc - n):rc + n + 1].any():
            return None        # already inside the lane: never stop there, keep going
        for px, py, d in self._route_points(pose, config.DYN_LANE_LOOKAHEAD):
            r, c = self.grid.world_to_cell(px, py)
            r0, r1 = max(0, r - n), min(self.grid.n_rows, r + n + 1)
            c0, c1 = max(0, c - n), min(self.grid.n_cols, c + n + 1)
            if r0 < r1 and c0 < c1 and lane[r0:r1, c0:c1].any():
                cross = (px, py)
                break
        if cross is None:
            self._lane_hist = []
            return None
        dist = np.hypot(ox - cross[0], oy - cross[1])
        near = dist <= config.DYN_LANE_RADIUS
        if not near.any():
            self._lane_hist = []
            return None
        dmin = float(dist[near].min())
        hist = [(t, d) for (t, d) in self._lane_hist if sim_time - t <= 1.0] + [(sim_time, dmin)]
        self._lane_hist = hist
        closing = 0.0
        if len(hist) >= 2 and hist[-1][0] - hist[0][0] > 0.2:
            closing = -(hist[-1][1] - hist[0][1]) / (hist[-1][0] - hist[0][0])
        if dmin < config.DYN_LANE_CLEAR_DIST:
            return cross
        if closing > 0.05:
            # will it reach the crossing before the robot has cleared it?
            d_robot = math.hypot(cross[0] - pose[0], cross[1] - pose[1])
            t_robot = (d_robot + config.BODY_LENGTH + 2 * config.ROBOT_RADIUS) / max(
                config.V_CRUISE * 0.7, 0.05)
            if (dmin - config.DYN_LANE_CLEAR_DIST) / closing < t_robot:
                return cross
        return None

    def _route_blocked(self, pose):
        """First point of the route (next REPLAN_LOOKAHEAD m) inside a dynamic
        obstacle grown by the robot radius, or None. Points inside the robot's
        own footprint are ignored."""
        fol = self.follower
        if fol is None or not fol.waypoints or self.grid is None:
            return None
        occ = self._dynamic_inflated()
        if not occ.any():
            return None
        pts = [(pose[0], pose[1])] + list(fol.waypoints[min(fol.idx, len(fol.waypoints) - 1):])
        travelled = 0.0
        step = self.grid.res * 0.5
        for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
            seg = math.hypot(bx - ax, by - ay)
            n = max(1, int(math.ceil(seg / step)))
            for k in range(1, n + 1):
                f = k / n
                px, py = ax + (bx - ax) * f, ay + (by - ay) * f
                d = travelled + seg * f
                if d > config.REPLAN_LOOKAHEAD:
                    return None
                if d < config.ROBOT_RADIUS:
                    continue
                r, c = self.grid.world_to_cell(px, py)
                if self.grid.in_bounds(r, c) and occ[r, c]:
                    return px, py
            travelled += seg
        return None

    def _replan(self, pose, goal, reason, sim_time, bypass_cooldown=False):
        """A* from the CURRENT pose to the SAME goal; recovery if impossible."""
        self._count_reason(reason)
        self.log(f"[t={sim_time:.1f}s] REPLANNING from ({pose[0]:.2f}, "
                 f"{pose[1]:.2f}) to ({goal[0]:.2f}, {goal[1]:.2f}) [trigger {reason}]...")
        wps, info = self.planner((pose[0], pose[1]), goal)
        self._last_replan_t = sim_time
        event = dict(t=round(sim_time, 2), reason=reason,
                     start=(round(pose[0], 3), round(pose[1], 3)),
                     goal=(round(goal[0], 3), round(goal[1], 3)),
                     ok=wps is not None and len(wps) > 0,
                     start_shift_m=round(info.get("start_shift_m", 0.0), 3),
                     why=info.get("reason", ""))
        self.replan_events.append(event)
        if event["ok"]:
            self.replans += 1
            self.path_length_total += info.get("length_m", 0.0)
            self._new_follower(wps)
            self.last_info = info
            self._blocked_start_time = None
            shift = info.get("start_shift_m", 0.0)
            note = (f", start moved {shift:.2f} m out of the inflated obstacle "
                    f"(straight move verified collision-free)" if shift > 0 else "")
            self.log(f"[t={sim_time:.1f}s] REPLAN SUCCESS: {info.get('n_waypoints', '?')} "
                     f"waypoints, length {info.get('length_m', 0):.2f} m, took "
                     f"{info.get('runtime_ms', 0):.1f} ms{note}")
            return 0.0, 0.0
        self.log(f"[t={sim_time:.1f}s] REPLAN FAILED: {info.get('reason', 'no path found')}")
        if self._nav_recoveries >= config.NAV_MAX_RECOVERIES:
            self.follower = None
            self.failure_reason = (f"NAVIGATION_FAILED: no path after "
                                   f"{self._nav_recoveries} recoveries ({reason})")
            self.log(f"ERROR: {self.failure_reason}")
            self._set_state("ERROR", sim_time)
            return 0.0, 0.0
        self._start_recovery(sim_time, f"REPLAN_FAILED/{reason}")
        return 0.0, 0.0

    def _start_recovery(self, sim_time, why):
        self._nav_recoveries += 1
        self.recoveries += 1
        cfg = RecoveryConfig(wait_timeout=1.5, rotate_timeout=4.0, backup_timeout=4.0,
                             replan_attempts=3, rotate_speed=0.5,
                             backup_speed=0.08,
                             backup_distance=config.RECOVERY_BACKUP_DIST,
                             # "no path" is not fixed by a clear front: run the
                             # full wait (map clears / obstacle moves), the full
                             # rotation and the backup before trying again
                             min_front_clearance=(math.inf if why.startswith("REPLAN_FAILED")
                                                  else 0.35),
                             min_rear_clearance=config.RECOVERY_BACKUP_DIST
                             + config.SAFETY_STOP_MARGIN)
        clock_t = {"t": sim_time}
        self._recovery = RecoveryBehavior(cfg, clock=lambda: clock_t["t"])
        self._recovery_clock = clock_t
        self._recovery.trigger()
        self._recovery_why = why
        self.log(f"[t={sim_time:.1f}s] RECOVERY start ({why}) — "
                 f"{self._nav_recoveries}/{config.NAV_MAX_RECOVERIES}: wait -> rotate "
                 f"if safe -> back up only if rear verified clear -> replan")

    def _recovery_step(self, pose, scan, angles, goal, sim_time, dt):
        rec = self._recovery
        self._recovery_clock["t"] = sim_time
        front = corridor_clearance(scan, angles, reverse=False)
        rear = corridor_clearance(scan, angles, reverse=True)
        rear_verified = self._rear_verified(scan, angles)
        esc = getattr(self, "_escape", None)
        if esc is not None:
            moved = math.hypot(pose[0] - esc["start"][0], pose[1] - esc["start"][1])
            side_clear = (rear_verified and rear > config.COLLISION_ESCAPE_DIST
                          + config.SAFETY_STOP_MARGIN) if esc["dir"] < 0 else \
                (front > config.COLLISION_ESCAPE_DIST + config.SAFETY_STOP_MARGIN)
            if (moved < config.COLLISION_ESCAPE_DIST and side_clear
                    and sim_time - esc["t0"] < 4.0):
                return esc["dir"] * 0.08, 0.0
            if not side_clear and moved < 0.01:
                self.log(f"[t={sim_time:.1f}s] RECOVERY: cannot move away from the "
                         f"contact ({'rear' if esc['dir'] < 0 else 'front'} not verified clear)")
            self._escape = None
            self._recovery_clock["t"] = sim_time
            rec._enter_state(rec.state)       # restart the WAIT timer after escaping
        rotate_safe = self._rotate_safe(scan)
        cmd = rec.update(dt, front, rear, pose[2], (pose[0], pose[1]),
                         rotate_safe=rotate_safe, rear_verified=rear_verified)
        st = rec.get_state()
        if st == RecoveryState.IDLE:
            # the obstacle went away while waiting: replan from here anyway
            # (the old route may still be marked) and resume
            self._recovery = None
            self.log(f"[t={sim_time:.1f}s] RECOVERY: path clear again")
            return self._replan(pose, goal, "RECOVERED", sim_time, bypass_cooldown=True)
        if st == RecoveryState.REPLAN:
            wps, info = self.planner((pose[0], pose[1]), goal)
            if wps:
                self._recovery = None
                self.log(f"[t={sim_time:.1f}s] RECOVERY: replanned from the current pose")
                return self._replan(pose, goal, "RECOVERED", sim_time, bypass_cooldown=True)
            rec.update(dt, front, rear, pose[2], (pose[0], pose[1]),
                       rotate_safe=rotate_safe, rear_verified=rear_verified)
            st = rec.get_state()
        if st == RecoveryState.ERROR:
            self._recovery = None
            if self._nav_recoveries < config.NAV_MAX_RECOVERIES:
                self._start_recovery(sim_time, self._recovery_why)
                return 0.0, 0.0
            self.follower = None
            self.failure_reason = f"NAVIGATION_FAILED: recovery exhausted ({self._recovery_why})"
            self.log(f"ERROR: {self.failure_reason}")
            self._set_state("ERROR", sim_time)
            return 0.0, 0.0
        if cmd is None:
            return 0.0, 0.0
        return cmd

    @staticmethod
    def _rear_verified(scan, angles):
        """Rear sector has valid LiDAR data (no NaN): reversing is only
        allowed on positive evidence, never on missing data."""
        if scan is None or angles is None:
            return False
        r = np.asarray(scan, dtype=float)
        a = np.asarray(angles, dtype=float)
        rear = np.abs(np.abs(a) - math.pi) <= math.radians(60)
        return bool(rear.any()) and not bool(np.isnan(r[rear]).any())

    @staticmethod
    def _rotate_safe(scan):
        if scan is None:
            return False
        r = np.asarray(scan, dtype=float)
        r = r[np.isfinite(r)]
        if r.size == 0:
            return True
        return float(r.min()) > config.ROBOT_SWEEP_RADIUS + config.RECOVERY_ROTATE_CLEAR

    def _handle_blocked(self, pose, scan, angles, goal, sim_time):
        """Follower safety stop: yield for moving obstacles, then replan from
        the current pose (recovery if no path)."""
        if self.grid is None:
            self.follower = None
            self._set_state("ERROR", sim_time)
            return 0.0, 0.0

        rx, ry, _ = pose
        gx, gy = goal
        dist_to_goal = math.hypot(rx - gx, ry - gy)

        # If already at/very close to target goal, consider reached
        if dist_to_goal < 0.35:
            if self.follower:
                self.follower.status = "REACHED"
            self._blocked_start_time = None
            return 0.0, 0.0

        yield_time = getattr(config, "YIELD_WAIT_TIME", 1.5)
        self._urgent = True             # follower LiDAR stop: obstacle within STOP_DIST
        if self.obstacle_map is not None and scan is not None and angles is not None:
            # a MOVING obstacle right in front: always let it pass first
            # (replanning around a crossing cart steers into its path)
            sc = np.asarray(scan, dtype=float)
            an = np.asarray(angles, dtype=float)
            ok = np.isfinite(sc)
            if ok.any():
                i = int(np.argmin(np.where(ok, sc, np.inf)))
                ox_ = pose[0] + sc[i] * math.cos(pose[2] + an[i])
                oy_ = pose[1] + sc[i] * math.sin(pose[2] + an[i])
                if self.obstacle_map.moving_near(ox_, oy_, 0.6, sim_time):
                    yield_time = max(yield_time, config.DYN_YIELD_TIME)

        # Dynamic yield: wait up to YIELD_WAIT_TIME for moving obstacle to cross
        if yield_time > 0.0:
            if self._blocked_start_time is None:
                self._blocked_start_time = sim_time
                self.log(f"[t={sim_time:.1f}s] Obstacle ahead — braking, yielding to let it pass...")

            elapsed = sim_time - self._blocked_start_time
            if elapsed < yield_time:
                from mapping import front_min_range, obstacle_in_sector
                if scan is not None and angles is not None:
                    # check clearance in the direction the follower was
                    # driving: it may have been REVERSING (bidirectional
                    # steering); checking only the front made a blocked
                    # reverse move loop forever between "yield" and "cleared"
                    if self.follower is not None and getattr(self.follower, "reversing", False):
                        front_d = obstacle_in_sector(scan, angles,
                                                     math.pi - config.FRONT_CONE,
                                                     -math.pi + config.FRONT_CONE)
                    else:
                        front_d = front_min_range(scan, angles, config.FRONT_CONE)
                    if front_d > (config.STOP_DIST + 0.12):
                        self.log(f"[t={sim_time:.1f}s] Obstacle cleared path after {elapsed:.1f}s — resuming shortest path.")
                        self._blocked_start_time = None
                        if self.follower:
                            self.follower.status = "FOLLOWING"
                        return 0.0, 0.0
                return 0.0, 0.0

        # Obstacle remained for > yield_time (or yield_time is 0) — replan detour
        self._blocked_start_time = None

        d_min = float("inf")
        phi_min = 0.0
        reversing = self.follower is not None and getattr(self.follower, "reversing", False)
        if scan is not None and angles is not None:
            for r, a in zip(scan, angles):
                rel = math.atan2(math.sin(a - (math.pi if reversing else 0.0)),
                                 math.cos(a - (math.pi if reversing else 0.0)))
                if abs(rel) <= config.FRONT_CONE and r < d_min:
                    d_min = r
                    phi_min = a

        if math.isinf(d_min):
            d_min = config.STOP_DIST
            phi_min = math.pi if reversing else 0.0

        ox = pose[0] + d_min * math.cos(pose[2] + phi_min)
        oy = pose[1] + d_min * math.sin(pose[2] + phi_min)

        # Do not mark the goal itself as an obstacle
        if math.hypot(ox - gx, oy - gy) < 0.40:
            if self.follower:
                self.follower.status = "REACHED"
            return 0.0, 0.0

        wp_idx = self.follower.idx if self.follower else 0
        self.log(f"[t={sim_time:.1f}s] OBSTACLE DETECTED at ({ox:.2f}, {oy:.2f}) distance {d_min:.2f} m")
        # restored Phase 12.1 trace line (dropped in "phase 14 + 15"; the
        # replanning test and the demo script grep for it)
        self.log(f"[t={sim_time:.1f}s] Current path BLOCKED at waypoint {wp_idx}")
        if self.obstacle_map is not None:
            # the continuous LiDAR layer already holds the obstacle's real
            # extent; refresh it now so the replan sees the latest scan
            self.obstacle_map.update(pose, scan, angles, sim_time, force=True,
                                     trusted=self.health.get("loc_ok", True))
            self._dyn_infl = None
        else:
            # legacy: mark the closest ray in the driving direction
            self.grid.add_obstacle_world(ox, oy, radius=0.12)
            r_row, r_col = self.grid.world_to_cell(pose[0], pose[1])
            if self.grid.in_bounds(r_row, r_col):
                self.grid.dynamic[r_row, r_col] = False
        # the same stop again from (almost) the same place: replanning alone
        # will not help (e.g. pushed against a known obstacle) -> recovery
        last = self._last_safety_stop
        self._last_safety_stop = (pose[0], pose[1], sim_time)
        if last is not None and math.hypot(pose[0] - last[0], pose[1] - last[1]) < 0.05:
            self._count_reason("SAFETY_STOP_REPEATED")
            self.log(f"[t={sim_time:.1f}s] Blocked again without progress — recovery")
            if self._nav_recoveries >= config.NAV_MAX_RECOVERIES:
                self.follower = None
                self.failure_reason = "NAVIGATION_FAILED: blocked without progress"
                self.log(f"ERROR: {self.failure_reason}")
                self._set_state("ERROR", sim_time)
                return 0.0, 0.0
            self._start_recovery(sim_time, "REPLAN_FAILED/SAFETY_STOP_REPEATED")
            return 0.0, 0.0
        return self._replan(pose, goal, "SAFETY_STOP", sim_time, bypass_cooldown=True)

    def _arrived_at_parcel(self, sim_time):
        """End of NAVIGATE: dock for a real pick, or the legacy placeholder."""
        if self.manip is not None:
            self._start_dock_pick(sim_time)
            return 0.0, 0.0
        self.pick_deadline = sim_time + config.PICK_PAUSE_S
        self._set_state("PICK", sim_time)
        return 0.0, 0.0

    def _do_pick(self, pose, scan, angles, detections, sim_time, dt):
        """LEGACY placeholder (no arm in the world): no physical pickup."""
        if sim_time >= self.pick_deadline:
            px, py = self.pkg_pos
            d = math.hypot(pose[0] - px, pose[1] - py)
            self.carrying = True
            self.log(f"[t={sim_time:.1f}s] Package {config.TASK_PACKAGE} — "
                     f"Distance: {d:.2f} m — picked up")
            self._set_state("PLAN_DELIVERY", sim_time)
        return 0.0, 0.0

    def _delivery_goal(self):
        """Navigation goal of DELIVER: zone centre (legacy) or slot pre-dock pose."""
        if self.manip is not None and self.target_slot is not None:
            gx, gy, _ = shelving.predock_pose(shelving.dock_line_for_slot(self.target_slot))
            return gx, gy
        return config.ZONE_OF[config.TASK_ZONE]

    def _do_plan_delivery(self, pose, scan, angles, detections, sim_time, dt):
        if self.manip is not None:
            self.target_slot = self.occupancy.next_free(config.TASK_ZONE)
            if self.target_slot is None:
                return self._mission_failed(
                    f"NO_FREE_SLOT on the Zone {config.TASK_ZONE} rack", sim_time,
                    recover=False)
            self.log(f"[t={sim_time:.1f}s] Destination: shelf slot "
                     f"{self.target_slot.slot_id} (Zone {config.TASK_ZONE} rack, "
                     f"level {self.target_slot.level}, bay {self.target_slot.bay})")
        goal = self._delivery_goal()
        self._begin_goal()
        wps, info = self.planner((pose[0], pose[1]), goal)
        self.last_info = info
        if wps is None:
            self._set_state("ERROR", sim_time)
            self.log("ERROR: no path to delivery zone")
            return 0.0, 0.0
        self.path_length_total += info.get("length_m", 0.0)
        self._new_follower(wps)
        self.log(f"[t={sim_time:.1f}s] A* path: {info.get('n_waypoints', '?')} "
                 f"waypoints, {info.get('length_m', 0):.2f} m, "
                 f"{info.get('runtime_ms', 0):.1f} ms, "
                 f"{info.get('expansions', '?')} expansions")
        self._set_state("DELIVER", sim_time)
        return 0.0, 0.0

    def _do_deliver(self, pose, scan, angles, detections, sim_time, dt):
        handled = self._nav_monitor(pose, scan, angles, self._delivery_goal(), sim_time, dt)
        if handled is not None:
            return handled
        v, w = self.follower.update(pose, scan, angles)
        if self.follower.status == "REACHED" and self.manip is not None:
            self.follower = None
            self._blocked_start_time = None
            self.docker = Docker(shelving.dock_line_for_slot(self.target_slot),
                                 log=self._dlog, name="DOCK_PLACE")
            self._set_state("DOCK_PLACE", sim_time)
            return 0.0, 0.0
        if self.follower.status == "REACHED":
            self.follower = None
            self.carrying = False
            self.log(f"[t={sim_time:.1f}s] Package {config.TASK_PACKAGE} "
                     f"dropped in Zone {config.TASK_ZONE}")
            self._emit_report(sim_time)
            # Interactive mode: return to center and loop; else finish.
            if getattr(config, "INTERACTIVE_MODE", False):
                self._set_state("RETURN_TO_CENTER", sim_time)
            else:
                self._set_state("DONE", sim_time)
        elif self.follower.status == "BLOCKED":
            self.log(f"[t={sim_time:.1f}s] BLOCKED during DELIVER (LiDAR safety stop)")
            goal = self._delivery_goal()
            return self._handle_blocked(pose, scan, angles, goal, sim_time)
        return v, w

    # ------------------------------------------------- new: interactive loop

    def _do_return_to_center(self, pose, scan, angles, detections, sim_time, dt):
        """Navigate back to the robot start position after a delivery."""
        if self.follower is None:
            cx, cy, _ = config.ROBOT_START
            rx, ry, _ = pose
            self._begin_goal()
            wps, info = self.planner((rx, ry), (cx, cy))
            self.last_info = info
            if wps is None:
                self.log("WARN: no path back to center — skipping to AWAIT_INPUT")
                self._set_state("AWAIT_INPUT", sim_time)
                return 0.0, 0.0
            self.path_length_total += info.get("length_m", 0.0)
            self._new_follower(wps)
            self.log(f"[t={sim_time:.1f}s] Returning to center: "
                     f"{info.get('n_waypoints', '?')} waypoints, "
                     f"{info.get('length_m', 0):.2f} m")

        cx, cy, _ = config.ROBOT_START
        handled = self._nav_monitor(pose, scan, angles, (cx, cy), sim_time, dt)
        if handled is not None:
            return handled
        v, w = self.follower.update(pose, scan, angles)
        if self.follower.status == "REACHED":
            self.follower = None
            self.log(f"[t={sim_time:.1f}s] Reached center — awaiting next task.")
            self._set_state("AWAIT_INPUT", sim_time)
        elif self.follower.status == "BLOCKED":
            self.log(f"[t={sim_time:.1f}s] BLOCKED during RETURN_TO_CENTER")
            cx, cy, _ = config.ROBOT_START
            return self._handle_blocked(pose, scan, angles, (cx, cy), sim_time)
        return v, w

    def _do_await_input(self, pose, scan, angles, detections, sim_time, dt):
        """Wait (non-blocking) for console input selecting the next task.

        A background thread reads stdin so the Webots step loop keeps running.
        The robot stays still.  When input is ready, transitions to IDLE.
        """
        if not self._awaiting_input:
            self._awaiting_input = True
            self._input_ready = False
            fn = (self._run_input_callback
                  if self.input_callback is not None
                  else self._console_input)
            threading.Thread(target=fn, daemon=True).start()

        if self._input_ready:
            self._awaiting_input = False
            self._input_ready = False
            if getattr(self, "_should_end", False):
                self._should_end = False
                self.log(f"[t={sim_time:.1f}s] Operator requested End Simulation.")
                self._set_state("DONE", sim_time)
                self._emit_report(sim_time)
                return 0.0, 0.0
            # Reset dynamic grid obstacles from the previous run so stale
            # package-obstacle marks don't block the new pickup route.
            if self.grid is not None:
                self.grid.clear_dynamic()
                if self.obstacle_map is not None:
                    self.obstacle_map.clear()
            self._set_state("IDLE", sim_time)

        return 0.0, 0.0

    def _run_input_callback(self):
        try:
            self.input_callback()
        except Exception as exc:
            self.log(f"[INPUT] callback error: {exc}")
        self._input_ready = True

    def _console_input(self):
        """Show a tkinter popup dialog to select color and drop zone.

        Webots does NOT connect a keyboard-readable stdin to the controller
        process, so input() blocks forever.  A tkinter dialog creates its own
        OS window that the operator can click — it works regardless of how
        Webots routes the subprocess streams.
        """
        try:
            self._tkinter_dialog()
        except Exception as exc:
            # Fallback: if tkinter is unavailable, use a task file
            print(f"[ROBOT] tkinter unavailable ({exc}), falling back to task file.")
            self._file_input()

    def _tkinter_dialog(self):
        """Pop up a small Tk window to select package color and drop zone."""
        import tkinter as tk
        from tkinter import ttk

        color_to_pkg = {v: k for k, v in config.PACKAGE_COLORS.items()}
        valid_colors = list(config.PACKAGE_COLORS.values())
        valid_zones  = list(config.ZONE_OF.keys())
        zone_info    = {z: config.COLOR_ZONE.get(z, z) for z in valid_zones}

        result = {}   # filled by the Submit button

        root = tk.Tk()
        root.title("IntelliBot — Next Task")
        root.resizable(False, False)
        root.attributes("-topmost", True)   # always on top of Webots

        pad = dict(padx=10, pady=6)

        tk.Label(root, text="IntelliBot — Select Next Task",
                 font=("Helvetica", 13, "bold")).grid(row=0, column=0,
                 columnspan=2, **pad)

        # --- color ---
        tk.Label(root, text="Pick color:", font=("Helvetica", 11)).grid(
            row=1, column=0, sticky="w", **pad)
        color_var = tk.StringVar(value=valid_colors[0])
        color_frame = tk.Frame(root)
        color_frame.grid(row=1, column=1, sticky="w", padx=10)
        COLOR_HEX = {"red": "#e74c3c", "green": "#27ae60", "blue": "#2980b9"}
        for c in valid_colors:
            tk.Radiobutton(color_frame, text=c.capitalize(),
                           variable=color_var, value=c,
                           fg=COLOR_HEX.get(c, "black"),
                           font=("Helvetica", 11, "bold")).pack(side="left",
                           padx=4)

        # --- zone ---
        tk.Label(root, text="Drop zone:", font=("Helvetica", 11)).grid(
            row=2, column=0, sticky="w", **pad)
        zone_var = tk.StringVar(value=valid_zones[0])
        zone_frame = tk.Frame(root)
        zone_frame.grid(row=2, column=1, sticky="w", padx=10)
        ZONE_HEX = {"A": "#f1c40f", "B": "#00bcd4"}   # yellow / cyan
        for z in valid_zones:
            label = f"Zone {z}  ({zone_info[z]})"
            tk.Radiobutton(zone_frame, text=label,
                           variable=zone_var, value=z,
                           fg=ZONE_HEX.get(z, "black"),
                           font=("Helvetica", 11)).pack(side="left", padx=8)

        # --- action buttons: Start Mission vs End Simulation ---
        btn_frame = tk.Frame(root)
        btn_frame.grid(row=3, column=0, columnspan=2, pady=12)

        def on_submit():
            result["action"] = "start"
            result["color"] = color_var.get()
            result["zone"]  = zone_var.get()
            root.destroy()

        def on_end():
            result["action"] = "end"
            root.destroy()

        tk.Button(btn_frame, text="Start Mission", command=on_submit,
                  bg="#2ecc71", fg="white",
                  font=("Helvetica", 11, "bold"),
                  padx=12, pady=4).pack(side="left", padx=10)

        tk.Button(btn_frame, text="End Simulation", command=on_end,
                  bg="#e74c3c", fg="white",
                  font=("Helvetica", 11, "bold"),
                  padx=12, pady=4).pack(side="left", padx=10)

        # Center the window on screen
        root.update_idletasks()
        w, h = root.winfo_width(), root.winfo_height()
        sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
        root.geometry(f"+{(sw - w) // 2}+{(sh - h) // 2}")

        root.mainloop()   # blocks until the window is closed

        action = result.get("action", "start")
        if action == "end":
            self._should_end = True
            self._input_ready = True
            print("[ROBOT] Operator selected: End Simulation.", flush=True)
            return

        self._should_end = False
        color = result.get("color", valid_colors[0])
        zone  = result.get("zone",  valid_zones[0])

        config.TASK_PACKAGE_COLOR = color
        config.TASK_PACKAGE       = color_to_pkg[color]
        config.TASK_PACKAGE_DEF   = f"PKG_{config.TASK_PACKAGE}"
        config.TASK_ZONE          = zone
        print(f"[ROBOT] Task set: pick {color} ({config.TASK_PACKAGE}) "
              f"-> deliver to Zone {zone} ({zone_info[zone]})", flush=True)
        self._input_ready = True

    def _file_input(self):
        """Fallback: poll for a file 'next_task.txt' in the controller folder.

        Format (two lines):
            color: blue
            zone: B

        Create the file while the simulation is running; the robot will pick
        it up and delete it automatically.
        """
        import os, time
        task_file = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "next_task.txt")
        color_to_pkg = {v: k for k, v in config.PACKAGE_COLORS.items()}
        valid_colors = list(config.PACKAGE_COLORS.values())
        valid_zones  = list(config.ZONE_OF.keys())

        print(f"[ROBOT] Waiting for {task_file} ...", flush=True)
        print(f"[ROBOT] File format:  color: <red|green|blue>  "
              f"(then newline)  zone: <A|B>", flush=True)

        while True:
            if os.path.exists(task_file):
                try:
                    data = {}
                    with open(task_file) as fh:
                        for line in fh:
                            if ":" in line:
                                k, v = line.split(":", 1)
                                data[k.strip().lower()] = v.strip().lower()
                    action = data.get("action", "").lower()
                    if action in ("end", "quit", "stop"):
                        os.remove(task_file)
                        self._should_end = True
                        self._input_ready = True
                        print("[ROBOT] Task file requested End Simulation.", flush=True)
                        return
                    color = data.get("color", "")
                    zone  = data.get("zone", "").upper()
                    if color in valid_colors and zone in valid_zones:
                        os.remove(task_file)
                        config.TASK_PACKAGE_COLOR = color
                        config.TASK_PACKAGE       = color_to_pkg[color]
                        config.TASK_PACKAGE_DEF   = f"PKG_{config.TASK_PACKAGE}"
                        config.TASK_ZONE          = zone
                        print(f"[ROBOT] Task loaded from file: {color} -> Zone {zone}",
                              flush=True)
                        self._input_ready = True
                        return
                    else:
                        print(f"[ROBOT] Bad task file (color={color!r}, zone={zone!r}),"
                              f" ignoring.", flush=True)
                except Exception as exc:
                    print(f"[ROBOT] Task file error: {exc}", flush=True)
            time.sleep(0.5)

    # ------------------------------------------------------------- existing

    def _emit_report(self, sim_time):
        self.mission_report = {
            "time_s": sim_time - self.t_start,
            "path_length_m": self.path_length_total,
            "replans": self.replans,
            "replan_reasons": dict(self.replan_reasons),
            "replan_events": list(self.replan_events[-20:]),
            "nav_recoveries": self.recoveries,
            "collisions": self.health.get("collisions", "N/A (no bumper data)"),
        }
        if self.governor is not None:
            self.mission_report["safety"] = self.governor.summary()
        if self.manip is not None:
            self.mission_report.update({
                "pick_mode": "arm (latched parallel gripper)",
                "result": "DELIVERED" if self.delivery_verified else "FAILED",
                "package": config.TASK_PACKAGE,
                "zone": config.TASK_ZONE,
                "slot": self.target_slot.slot_id if self.target_slot else None,
                "grasp_attempts": self.grasp_attempts,
                "place_attempts": self.place_attempts,
                "delivery_verified_by_sensors": self.delivery_verified,
                "failure_reason": self.failure_reason,
                "docking": list(self.dock_log),
                "manipulation_checks": dict(self.manip.last_check),
            })
        else:
            self.mission_report["pick_mode"] = "placeholder (no arm in this world)"
        self.log(f"[t={sim_time:.1f}s] MISSION REPORT: "
                 f"time={self.mission_report['time_s']:.1f}s, "
                 f"path_length={self.mission_report['path_length_m']:.2f} m, "
                 f"replans={self.mission_report['replans']}, "
                 f"collisions={self.mission_report['collisions']}")
        if self.manip is not None:
            r = self.mission_report
            self.log(f"[t={sim_time:.1f}s] MANIPULATION: result={r['result']}, "
                     f"slot={r['slot']}, grasp_attempts={r['grasp_attempts']}, "
                     f"place_attempts={r['place_attempts']}, "
                     f"verified_by_sensors={r['delivery_verified_by_sensors']}, "
                     f"failure={r['failure_reason']}")

    # ======================================================= manipulation
    def _dlog(self, msg):
        self.log(msg)

    def _record_dock(self, docker):
        self.dock_log.append({
            "name": docker.name, "status": docker.status, "reason": docker.reason,
            "range_m": None if docker.measured_range is None else round(docker.measured_range, 4),
            "range_source": docker.range_source,
            "lateral_err_m": (None if docker.final_lateral_error is None
                              else round(docker.final_lateral_error, 4)),
            "heading_err_deg": (None if docker.final_heading_error is None
                                else round(math.degrees(docker.final_heading_error), 2)),
        })

    def _start_dock_pick(self, sim_time):
        # grasp the face that looks towards the planned approach pose (the
        # open aisle), not whichever face the robot happens to see first
        preferred = None
        if self.approach_pose is not None and self.pkg_pos is not None:
            preferred = math.atan2(self.pkg_pos[1] - self.approach_pose[1],
                                   self.pkg_pos[0] - self.approach_pose[0])
        self.docker = Docker(None, parcel_estimate=self.pkg_pos,
                             preferred_heading=preferred, log=self._dlog,
                             name="DOCK_PICK")
        self._set_state("DOCK_PICK", sim_time)

    def _reachable(self, targets):
        """All (x, z) TCP targets inside the arm workspace and joint limits?"""
        if self.kin is None:
            return True, ""
        for x, z in targets:
            q, reason = self.kin.solve(x, z, 0.0, elbow_up=True)
            if q is None:
                return False, f"({x:.3f}, {z:.3f}) {reason}"
        return True, ""

    def _do_dock_pick(self, pose, scan, angles, detections, sim_time, dt):
        v, w = self.docker.update(pose, scan, angles, sim_time)
        if self.docker.status == "RUNNING":
            return v, w
        self._record_dock(self.docker)
        if self.docker.status == "FAILED":
            return self._pick_failed(f"DOCK_PICK {self.docker.reason}", sim_time,
                                     arm_moved=False)
        if getattr(self.docker, "refined_center", None) is not None:
            self.pkg_pos = self.docker.refined_center
        grasp = shelving.pick_tcp_for_parcel(self.docker.measured_range)
        ok, why = self._reachable([
            (grasp["x"], grasp["z"]),
            (grasp["x"] - config.ARM_PREGRASP_BACKOFF, grasp["z"]),
            (grasp["x"] - config.ARM_PREGRASP_BACKOFF,
             grasp["z"] + config.ARM_PREGRASP_RAISE)])
        if not ok:
            return self._pick_failed(f"PARCEL_OUT_OF_REACH {why}", sim_time,
                                     arm_moved=False)
        self.grasp_attempts += 1
        self.log(f"[t={sim_time:.1f}s] Docked at parcel: face range "
                 f"{self.docker.measured_range:.3f} m -> grasp TCP "
                 f"({grasp['x']:.3f}, {grasp['z']:.3f}) m in the robot frame, "
                 f"attempt {self.grasp_attempts}/{config.ARM_MAX_GRASP_ATTEMPTS}")
        self.manip.start_pick(sim_time, grasp)
        self._set_state("ARM_PICK", sim_time)
        return 0.0, 0.0

    def _do_arm_pick(self, pose, scan, angles, detections, sim_time, dt):
        st = self.manip.update(sim_time)
        if st == "SUCCEEDED":
            self.manip.start_verify_attachment(sim_time)
            self._set_state("VERIFY_ATTACHMENT", sim_time)
        elif st == "FAILED":
            return self._pick_failed(self.manip.reason, sim_time)
        return 0.0, 0.0

    def _do_verify_attachment(self, pose, scan, angles, detections, sim_time, dt):
        st = self.manip.update(sim_time)
        if st == "SUCCEEDED":
            self.carrying = True
            px, py = self.pkg_pos
            d = math.hypot(pose[0] - px, pose[1] - py)
            self.log(f"[t={sim_time:.1f}s] Package {config.TASK_PACKAGE} — "
                     f"Distance: {d:.2f} m — picked up by the arm "
                     f"(attachment verified: width + palm range + force/latch)")
            self._set_state("PLAN_DELIVERY", sim_time)
        elif st == "FAILED":
            return self._pick_failed(self.manip.reason, sim_time)
        return 0.0, 0.0

    def _pick_failed(self, reason, sim_time, arm_moved=True):
        """Recover and retry the pick, or give up with ERROR."""
        self.log(f"[t={sim_time:.1f}s] PICK FAILED: {reason}")
        self.failure_reason = reason
        retry = self.grasp_attempts < config.ARM_MAX_GRASP_ATTEMPTS
        if not arm_moved:
            # docking failed or target unreachable: count it as an attempt
            self.grasp_attempts += 1
            retry = self.grasp_attempts < config.ARM_MAX_GRASP_ATTEMPTS
            if retry:
                self.log(f"[t={sim_time:.1f}s] Retrying the docking "
                         f"({self.grasp_attempts}/{config.ARM_MAX_GRASP_ATTEMPTS})")
                self._start_dock_pick(sim_time)
                return 0.0, 0.0
            return self._mission_failed(reason, sim_time, recover=False)
        self.carrying = False
        self._recover_then("DOCK_PICK" if retry else "ERROR", sim_time,
                           keep_parcel=False)
        return 0.0, 0.0

    def _do_dock_place(self, pose, scan, angles, detections, sim_time, dt):
        v, w = self.docker.update(pose, scan, angles, sim_time)
        if self.docker.status == "RUNNING":
            return v, w
        self._record_dock(self.docker)
        self.place_attempts += 1
        if self.docker.status == "FAILED":
            if self.place_attempts < config.ARM_MAX_GRASP_ATTEMPTS:
                self.log(f"[t={sim_time:.1f}s] DOCK_PLACE failed "
                         f"({self.docker.reason}) — retrying")
                self.docker = Docker(shelving.dock_line_for_slot(self.target_slot),
                                     log=self._dlog, name="DOCK_PLACE")
                return 0.0, 0.0
            return self._mission_failed(f"DOCK_PLACE {self.docker.reason}", sim_time,
                                        recover=False)
        place = shelving.place_tcp_for_slot(self.target_slot,
                                            measured_range=self.docker.measured_range)
        ok, why = self._reachable([(place["pre_x"], place["z_hover"]),
                                   (place["x"], place["z_hover"]),
                                   (place["x"], place["z_release"]),
                                   (place["retract_x"], place["z_release"])])
        if not ok:
            return self._mission_failed(f"SHELF_TARGET_UNREACHABLE {why}", sim_time,
                                        recover=False)
        self.place_targets = place
        self.log(f"[t={sim_time:.1f}s] Docked at rack: front range "
                 f"{self.docker.measured_range:.3f} m -> place TCP "
                 f"({place['x']:.3f}, {place['z_release']:.3f}) m in the robot frame "
                 f"for slot {self.target_slot.slot_id}")
        self.manip.start_place(sim_time, place)
        self._set_state("ARM_PLACE", sim_time)
        return 0.0, 0.0

    def _do_arm_place(self, pose, scan, angles, detections, sim_time, dt):
        st = self.manip.update(sim_time)
        if st == "SUCCEEDED":
            self.carrying = False           # latch open, fingers open
            self.manip.start_verify_delivery(sim_time, self.place_targets)
            self._set_state("VERIFY_DELIVERY", sim_time)
        elif st == "FAILED":
            reason = self.manip.reason
            self.failure_reason = reason
            self.log(f"[t={sim_time:.1f}s] PLACE FAILED: {reason}")
            if "SLOT_BLOCKED" in reason:
                # something occupies this slot: keep the parcel, back out,
                # mark the slot and try the next free one
                self.occupancy.mark(self.target_slot.slot_id, "blocked")
                self._recover_then("PLAN_DELIVERY", sim_time, keep_parcel=True,
                                   retreat_x=self.place_targets["pre_x"])
            else:
                self._recover_then("ERROR", sim_time, keep_parcel=True,
                                   retreat_x=self.place_targets["pre_x"])
        return 0.0, 0.0

    def _do_verify_delivery(self, pose, scan, angles, detections, sim_time, dt):
        st = self.manip.update(sim_time)
        if st == "SUCCEEDED":
            self.delivery_verified = True
            self.failure_reason = None
            self.occupancy.mark(self.target_slot.slot_id, config.TASK_PACKAGE)
            self.log(f"[t={sim_time:.1f}s] Package {config.TASK_PACKAGE} placed on "
                     f"shelf slot {self.target_slot.slot_id} — dropped in Zone "
                     f"{config.TASK_ZONE} rack, delivery verified by the palm "
                     f"range-finder")
            self._emit_report(sim_time)
            if getattr(config, "INTERACTIVE_MODE", False):
                self._set_state("RETURN_TO_CENTER", sim_time)
            else:
                self._set_state("DONE", sim_time)
        elif st == "FAILED":
            self._mission_failed(f"DELIVERY_NOT_VERIFIED {self.manip.reason}",
                                 sim_time, recover=True)
        return 0.0, 0.0

    def _recover_then(self, next_state, sim_time, keep_parcel=False, retreat_x=None):
        self._after_recover = next_state
        self.manip.start_recover(sim_time, keep_parcel=keep_parcel,
                                 retreat_x=retreat_x)
        self._set_state("ARM_RECOVER", sim_time)

    def _do_arm_recover(self, pose, scan, angles, detections, sim_time, dt):
        st = self.manip.update(sim_time)
        if st == "RUNNING":
            return 0.0, 0.0
        nxt = self._after_recover
        self._after_recover = None
        if st == "FAILED":
            self.failure_reason = (f"{self.failure_reason} + ARM_RECOVERY_FAILED "
                                   f"{self.manip.reason}")
            nxt = "ERROR"
        if nxt == "DOCK_PICK":
            self._start_dock_pick(sim_time)
        elif nxt == "PLAN_DELIVERY":
            self._set_state("PLAN_DELIVERY", sim_time)
        else:
            self._enter_error(sim_time)
        return 0.0, 0.0

    def _mission_failed(self, reason, sim_time, recover=True):
        """Terminal failure: never report success; stow the arm first if asked."""
        self.failure_reason = reason
        self.log(f"[t={sim_time:.1f}s] MISSION FAILED: {reason}")
        if recover and self.manip is not None:
            self._recover_then("ERROR", sim_time, keep_parcel=False)
            return 0.0, 0.0
        self._enter_error(sim_time)
        return 0.0, 0.0

    def _enter_error(self, sim_time):
        self._set_state("ERROR", sim_time)
        self.log(f"ERROR: {self.failure_reason}")
        if self.manip is not None:
            self._emit_report(sim_time)

    def _do_done(self, pose, scan, angles, detections, sim_time, dt):
        if self.mission_report is None:
            self._emit_report(sim_time)
        return 0.0, 0.0

    def _do_error(self, pose, scan, angles, detections, sim_time, dt):
        return 0.0, 0.0
