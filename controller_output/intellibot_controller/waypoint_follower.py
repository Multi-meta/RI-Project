"""waypoint_follower.py — WaypointFollower: PID heading control along waypoints.

Pure logic (no Webots): per step it takes the robot pose and the current
LiDAR scan, and returns the commanded body twist (v, w).

Control law (project_summary.md §10.6):
    dist        = |wp - pos|
    heading_err = wrap_to_pi(atan2(dy, dx) - theta)
    w           = PID(heading_err), clamped to +-W_MAX
    v           = V_MAX * clamp(cos(heading_err), 0, 1) * min(1, dist/SLOWDOWN)
    if |heading_err| > TURN_IN_PLACE_THRESH: v = 0
    waypoint reached when dist < WP_TOLERANCE (last: GOAL_TOLERANCE)

Safety: if any LiDAR ray in the forward cone (+-30 deg) is closer than
STOP_DIST, v = 0 and status = BLOCKED. This BLOCKED flag is the hook the
final-eval re-planner will use.

status: FOLLOWING | BLOCKED | REACHED
"""

import math

import config
from kinematics import wrap_to_pi
from mapping import front_min_range
from pid import PID


class WaypointFollower:
    def __init__(self, waypoints, v_max=None, w_max=None,
                 wp_tol=None, goal_tol=None, slow_dist=None,
                 turn_thresh=None, stop_dist=None, front_cone=None,
                 pid_gains=None):
        self.waypoints = [(float(x), float(y)) for x, y in waypoints]
        self.idx = 0
        self.status = "FOLLOWING"
        self.cross_track_err = 0.0
        self.last_heading_err = 0.0

        self.v_max = v_max if v_max is not None else config.V_MAX
        self.w_max = w_max if w_max is not None else config.W_MAX
        self.wp_tol = wp_tol if wp_tol is not None else config.WP_TOLERANCE
        self.goal_tol = goal_tol if goal_tol is not None else config.GOAL_TOLERANCE
        self.slow = slow_dist if slow_dist is not None else config.SLOWDOWN_DIST
        self.turn_thresh = (turn_thresh if turn_thresh is not None
                            else config.TURN_IN_PLACE_THRESH)
        self.stop_dist = stop_dist if stop_dist is not None else config.STOP_DIST
        self.front_cone = front_cone if front_cone is not None else config.FRONT_CONE
        gains = pid_gains if pid_gains is not None else config.HEADING_PID
        self.pid = PID(gains["kp"], gains["ki"], gains["kd"],
                       out_min=-self.w_max, out_max=self.w_max)

    @property
    def current_waypoint(self):
        if self.idx < len(self.waypoints):
            return self.waypoints[self.idx]
        return self.waypoints[-1] if self.waypoints else None

    @property
    def done(self):
        return self.status == "REACHED"

    def update(self, pose, scan=None, angles=None):
        """One control step. pose=(x,y,theta); scan/angles = LiDAR (optional).

        Returns (v, w) body command. Advances waypoints as they are reached.
        """
        x, y, theta = pose
        if not self.waypoints:
            self.status = "REACHED"
            return 0.0, 0.0
        if self.status == "REACHED":
            return 0.0, 0.0

        # --- LiDAR safety stop (hook for final-eval replanning) -----------
        if scan is not None and angles is not None:
            front = front_min_range(scan, angles, self.front_cone)
            if front < self.stop_dist:
                self.status = "BLOCKED"
                return 0.0, 0.0

        wp_x, wp_y = self.waypoints[self.idx]
        dx, dy = wp_x - x, wp_y - y
        dist = math.hypot(dx, dy)

        heading_err = wrap_to_pi(math.atan2(dy, dx) - theta)
        self.last_heading_err = heading_err

        tol = self.goal_tol if self.idx == len(self.waypoints) - 1 else self.wp_tol
        if dist < tol:
            self.idx += 1
            if self.idx >= len(self.waypoints):
                self.status = "REACHED"
                return 0.0, 0.0
            wp_x, wp_y = self.waypoints[self.idx]
            dx, dy = wp_x - x, wp_y - y
            dist = math.hypot(dx, dy)
            heading_err = wrap_to_pi(math.atan2(dy, dx) - theta)
            self.last_heading_err = heading_err

        # cross-track error: perpendicular distance to the current segment
        self.cross_track_err = self._crosstrack(x, y)

        w = self.pid.update(heading_err, 1.0 / 60.0)  # nominal dt for pure tests
        w = max(-self.w_max, min(self.w_max, w))

        if abs(heading_err) > self.turn_thresh:
            v = 0.0                                   # rotate first
        else:
            v = self.v_max * max(0.0, math.cos(heading_err)) \
                * min(1.0, dist / self.slow)
        return v, w

    def _crosstrack(self, x, y):
        """Distance from (x, y) to the line through the previous waypoint and
        the current one (0 for the first waypoint)."""
        if self.idx == 0:
            return 0.0
        ax, ay = self.waypoints[self.idx - 1]
        bx, by = self.waypoints[self.idx]
        vx, vy = bx - ax, by - ay
        seg2 = vx * vx + vy * vy
        if seg2 < 1e-12:
            return math.hypot(x - bx, y - by)
        # signed cross product / segment length
        return abs((x - ax) * vy - (y - ay) * vx) / math.sqrt(seg2)
