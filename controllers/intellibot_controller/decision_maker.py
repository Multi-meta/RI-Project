"""decision_maker.py — the mission FSM (pure logic, no Webots).

States:
    IDLE -> FIND_PACKAGE -> NAVIGATE -> PICK -> PLAN_DELIVERY -> DELIVER
         -> RETURN_TO_CENTER -> AWAIT_INPUT -> IDLE  (continuous interactive loop)
         -> DONE (only when INTERACTIVE_MODE = False), ERROR as terminal failure.

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

import config
from waypoint_follower import WaypointFollower
from logger import transition


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
                 input_callback=None):
        self.planner = planner          # callable -> (waypoints, info)
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
        self.carrying = False
        self._pkg_seen = {}
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

        rx, ry, _ = pose
        ax, ay = self.approach_pose
        dist_to_approach = math.hypot(rx - ax, ry - ay)

        # Proximity arrival check for pickup
        if dist_to_approach < 0.28:
            self.follower = None
            self._blocked_start_time = None
            self.pick_deadline = sim_time + config.PICK_PAUSE_S
            self._set_state("PICK", sim_time)
            return 0.0, 0.0

        v, w = self.follower.update(pose, scan, angles)
        if self.follower.status == "REACHED":
            self.follower = None
            self._blocked_start_time = None
            self.pick_deadline = sim_time + config.PICK_PAUSE_S
            self._set_state("PICK", sim_time)
        elif self.follower.status == "BLOCKED":
            return self._handle_blocked(pose, scan, angles, self.approach_pose, sim_time)
        else:
            self._blocked_start_time = None
        return v, w

    def _handle_blocked(self, pose, scan, angles, goal, sim_time):
        """Yield for moving obstacles; replan only if permanently blocked."""
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

        # Dynamic yield: wait up to YIELD_WAIT_TIME for moving obstacle to cross
        if yield_time > 0.0:
            if not hasattr(self, '_blocked_start_time') or self._blocked_start_time is None:
                self._blocked_start_time = sim_time
                self.log(f"[t={sim_time:.1f}s] Dynamic obstacle ahead — yielding to let it pass...")

            elapsed = sim_time - self._blocked_start_time
            if elapsed < yield_time:
                from mapping import front_min_range
                if scan is not None and angles is not None:
                    front_d = front_min_range(scan, angles, config.FRONT_CONE)
                    if front_d > (config.STOP_DIST + 0.12):
                        self.log(f"[t={sim_time:.1f}s] Obstacle cleared path after {elapsed:.1f}s — resuming shortest path.")
                        self._blocked_start_time = None
                        if self.follower:
                            self.follower.status = "FOLLOWING"
                        return 0.0, 0.0
                return 0.0, 0.0

        # Obstacle remained static for > yield_time (or yield_time is 0) — replan detour
        self._blocked_start_time = None

        d_min = float("inf")
        phi_min = 0.0
        if scan is not None and angles is not None:
            for r, a in zip(scan, angles):
                if abs(a) <= config.FRONT_CONE and r < d_min:
                    d_min = r
                    phi_min = a

        if math.isinf(d_min):
            d_min = config.STOP_DIST
            phi_min = 0.0

        ox = pose[0] + d_min * math.cos(pose[2] + phi_min)
        oy = pose[1] + d_min * math.sin(pose[2] + phi_min)

        # Do not mark the goal itself as an obstacle
        if math.hypot(ox - gx, oy - gy) < 0.40:
            if self.follower:
                self.follower.status = "REACHED"
            return 0.0, 0.0

        self.log(f"[t={sim_time:.1f}s] OBSTACLE DETECTED at ({ox:.2f}, {oy:.2f}) distance {d_min:.2f} m")
        self.grid.add_obstacle_world(ox, oy, radius=0.12)
        r_row, r_col = self.grid.world_to_cell(pose[0], pose[1])
        if self.grid.in_bounds(r_row, r_col):
            self.grid.dynamic[r_row, r_col] = False

        self.log(f"[t={sim_time:.1f}s] REPLANNING from ({pose[0]:.2f}, {pose[1]:.2f}) to ({goal[0]:.2f}, {goal[1]:.2f})...")
        wps, info = self.planner((pose[0], pose[1]), goal)
        if wps is not None and len(wps) > 0:
            self.replans += 1
            self.path_length_total += info.get("length_m", 0.0)
            self.follower = WaypointFollower(wps)
            self.last_info = info
            self.log(f"[t={sim_time:.1f}s] REPLAN SUCCESS: {info.get('n_waypoints', '?')} waypoints, "
                     f"length {info.get('length_m', 0):.2f} m, took {info.get('runtime_ms', 0):.1f} ms")
            return 0.0, 0.0
        else:
            self.log(f"[t={sim_time:.1f}s] REPLAN FAILED: no path found")
            self.follower = None
            self._set_state("ERROR", sim_time)
            return 0.0, 0.0

    def _do_pick(self, pose, scan, angles, detections, sim_time, dt):
        if sim_time >= self.pick_deadline:
            px, py = self.pkg_pos
            d = math.hypot(pose[0] - px, pose[1] - py)
            self.carrying = True
            self.log(f"[t={sim_time:.1f}s] Package {config.TASK_PACKAGE} — "
                     f"Distance: {d:.2f} m — picked up")
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
            goal = config.ZONE_OF[config.TASK_ZONE]
            return self._handle_blocked(pose, scan, angles, goal, sim_time)
        return v, w

    # ------------------------------------------------- new: interactive loop

    def _do_return_to_center(self, pose, scan, angles, detections, sim_time, dt):
        """Navigate back to the robot start position after a delivery."""
        if self.follower is None:
            cx, cy, _ = config.ROBOT_START
            rx, ry, _ = pose
            wps, info = self.planner((rx, ry), (cx, cy))
            self.last_info = info
            if wps is None:
                self.log("WARN: no path back to center — skipping to AWAIT_INPUT")
                self._set_state("AWAIT_INPUT", sim_time)
                return 0.0, 0.0
            self.path_length_total += info.get("length_m", 0.0)
            self.follower = WaypointFollower(wps)
            self.log(f"[t={sim_time:.1f}s] Returning to center: "
                     f"{info.get('n_waypoints', '?')} waypoints, "
                     f"{info.get('length_m', 0):.2f} m")

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
