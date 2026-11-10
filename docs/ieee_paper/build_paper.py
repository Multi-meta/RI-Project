"""build_paper.py: IEEE two-column version of the IntelliBot project report.

One content definition (CONTENT below) generates:
  * paper.tex  : IEEEtran conference source (two columns). Compile on Overleaf
                 or with any TeX install:  pdflatex paper.tex (twice).
  * paper.html : the same paper laid out in IEEE two-column style with KaTeX
                 math, printed to paper.pdf by Edge/Chrome headless.

    python docs/ieee_paper/build_paper.py          # writes .tex + .html + .pdf

Inline markup inside text: $math$, `code`, **bold**, *italic*.
"""

import html
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

TITLE = "IntelliBot: Autonomous Mobile Manipulator for Warehouse Pick-and-Place"
AUTHORS = ["Abhishek Anand", "Ankit Kumar", "Utkarsh Yuvraj"]
AFFILIATION = "Robotic Intelligence, course project"
REPO = "https://github.com/Multi-meta/RI-Project"

ABSTRACT = (
    "IntelliBot completes a full pick-and-place mission in simulation: it finds parcel P3 with "
    "its cameras, picks it with a 3-joint arm, carries it past moving traffic and places it on the "
    "Zone B rack in 72.9 s, 0.6 cm from the slot centre, with zero contacts. We built a miniature "
    "warehouse in the Webots simulator and a differential-drive mobile manipulator for it, carrying "
    "a 360-ray LiDAR, front and rear cameras, GPS, an IMU, wheel encoders and two bumpers. "
    "Everything runs in a closed loop in Python: a colour detector finds the parcel, A* plans a route "
    "on an inflated occupancy grid, a PID waypoint follower drives it, and a finite-state machine "
    "sequences the mission. Three features make the robot robust rather than scripted: an Extended "
    "Kalman Filter (EKF) fuses odometry, GPS and IMU, gates out bad readings and falls back to dead "
    "reckoning when GPS is lost; a dynamic obstacle layer built from LiDAR triggers replanning from "
    "the current pose; and a speed governor limits velocity to what the robot can stop within, given "
    "the measured clearance. In Webots the EKF tracks ground truth with 5 mm position and "
    "0.12$^\\circ$ heading RMSE, and a Webots memory leak that crashed long runs was found and "
    "fixed."
)
KEYWORDS = ("mobile manipulation, extended Kalman filter, sensor fusion, A* path planning, "
            "warehouse automation, Webots")

# ----------------------------------------------------------------------- content
# ("sec", title) ("sub", title) ("p", text) ("ul", [..]) ("ol", [..])
# ("eq", latex, label) ("fig", file, caption, wide) ("tab", caption, header, rows, colspec, wide)
CONTENT = [
    ("sec", "Introduction"),
    ("p", "Warehouse robots must find goods, move them through shared aisles and put them away "
          "without a human steering them. This project builds that capability end to end, in "
          "simulation, at a scale a student team can explain line by line."),
    ("sub", "Motivation"),
    ("p", "Autonomous mobile robots now move a large share of goods in fulfilment centres. The hard "
          "part is not driving a known route but staying correct when the world disagrees with the "
          "plan: a parcel is not exactly where expected, a cart blocks the aisle, or a sensor drops out. "
          "A robot that only replays a recorded path fails at the first "
          "surprise. We wanted a robot whose every action comes from what it senses in the moment."),
    ("sub", "Problem Statement"),
    ("p", "Given the task \u201cpick package P3 and deliver it to Zone B\u201d, the robot must:"),
    ("ol", ["find the parcel with its own sensors, not from a map entry;",
            "plan a collision-free route on a known warehouse map;",
            "drive the route while avoiding static and moving obstacles;",
            "dock precisely enough for an arm to grasp a 20 cm parcel;",
            "carry it to the destination rack and place it in a free slot;",
            "verify the result and write a mission report."]),
    ("p", "A hard-coded route or a teleported parcel is explicitly not acceptable: the parcel must "
          "be held by the gripper and moved by physics."),
    ("sub", "Objectives"),
    ("ul", ["Model a warehouse world and a mobile manipulator in Webots [1], generated from one "
            "configuration file so the map and the world can never disagree.",
            "Implement differential-drive kinematics, odometry and PID heading control [4], [10], "
            "and validate them against ground truth.",
            "Detect parcels by colour with OpenCV [8] and estimate their position from camera "
            "bearing plus LiDAR range.",
            "Plan with A* [3] on an inflated occupancy grid and replan from the current pose when "
            "the route is blocked.",
            "Fuse odometry, GPS and IMU with an EKF [2] that rejects outliers and degrades "
            "gracefully.",
            "Bound speed by stopping distance so the robot never outruns its sensors.",
            "Pick and place with a 3-joint arm [5] and a latching gripper, with sensor-based "
            "verification.",
            "Test robustness with moving obstacles, and report measured "
            "numbers only."]),
    ("sub", "Scope"),
    ("p", "The work is simulation only. Localisation uses GPS, IMU and wheel odometry; there is no "
          "SLAM. One robot works at a time; the multi-robot world exists but was not evaluated. All "
          "results are from Webots runs or from an offline kinematic simulator, and each number "
          "says which."),
    ("sub", "Paper Outline"),
    ("p", "Section II describes the world, the robot and the software. Section III presents "
          "measured results with graphs and simulator screenshots, and discusses the problems we "
          "found. Sections IV and V conclude and list future work."),

    ("sec", "Methodology: Working Model, Simulation and Code"),
    ("p", "The robot runs a sense\u2013decide\u2013act loop every 16 ms: sensors feed perception "
          "and the EKF, a state machine picks the next action, a planner and controller turn it "
          "into wheel and arm commands, and every step is logged."),
    ("sub", "Simulation World"),
    ("p", "The warehouse is an 8 m $\\times$ 6 m walled floor in Webots R2025a (16 ms physics step, "
          "ENU frame). `tools/generate_world.py` writes the world file from `config.py`, so the "
          "occupancy grid the robot plans on and the world it drives in come from the same numbers. "
          "A supervisor check compares object positions with the configuration at start-up and "
          "warns on any mismatch over 1 cm. Table I lists the world elements and Fig. 1 shows the "
          "generated world."),
    ("tab", "Simulation world elements",
     ["Element", "Position / size", "Purpose"],
     [["4 shelves", "1.6 $\\times$ 0.5 $\\times$ 1.0 m at ($\\pm$2.0, $\\pm$1.9)", "static aisles"],
      ["Crates, barrels, ball", "0.3\u20130.4 m boxes; $r$ = 0.12\u20130.15 m cylinders; $r$ = 0.15 m sphere",
       "varied obstacle shapes"],
      ["Pickup area", "1.2 $\\times$ 1.6 m at ($-$3.0, 0.0)",
       "parcels P1 (red), P2 (green), P3 (blue): 0.2 m cubes, 0.5 kg"],
      ["Zone A / Zone B", "1 $\\times$ 1 m patches at (3.0, $\\pm$0.9), yellow / cyan", "dispatch zones"],
      ["Destination racks A / B", "east wall; levels at 0.15, 0.41, 0.67 m",
       "placement slots (L1, L2 reachable)"],
      ["CART\\_1, CART\\_2", "patrol $y$ = 2.4 at 0.4 m/s; cross aisle $x$ = 0.72 at 0.3 m/s",
       "moving traffic"],
      ["DECOY\\_BOT", "yellow look-alike robot, $x = -1.5$, $y \\in [-1.20, 0.15]$, 0.25 m/s",
       "crosses the start-to-pickup route and gives way"]],
     "p{0.2\\linewidth}p{0.45\\linewidth}p{0.27\\linewidth}", True),
    ("fig", "figures/fig1_world.png",
     "The generated warehouse in Webots: pickup area with parcels P1\u2013P3 (left), shelves, varied "
     "obstacles, Zone A (yellow) and Zone B (cyan) with destination racks (right), and IntelliBot "
     "(blue). Inset: the robot's front camera.", False),
    ("sub", "Robot Model"),
    ("p", "IntelliBot is a differential-drive base (0.30 $\\times$ 0.20 $\\times$ 0.10 m, 2.8 kg) "
          "with two driven wheels ($r$ = 0.05 m, track $L$ = 0.24 m, 20 rad/s limit) and two "
          "zero-friction casters. A post at the rear carries the arm. Table II lists its devices."),
    ("tab", "Robot devices",
     ["Device", "Specification", "Used for"],
     [["LiDAR", "360 rays, 360$^\\circ$, 0.05\u20134.0 m, 0.14 m high", "obstacles, docking, dynamic map"],
      ["Front and rear cameras", "320 $\\times$ 240, 1.0 rad planar FOV, 0.12 m high",
       "parcel detection, camera health"],
      ["GPS + IMU", "position $(x, y)$ and yaw", "EKF corrections"],
      ["Wheel encoders", "position sensors on both wheels", "odometry (EKF prediction)"],
      ["Front and rear bumpers", "touch sensors", "collision events and recovery"],
      ["Arm", "shoulder, elbow, wrist pitch joints; links 0.28 m and 0.25 m",
       "pick from floor, place on rack"],
      ["Gripper", "two fingers, 0.12 m travel each, force pads, palm range sensor, Connector latch",
       "grasp, verify, hold the parcel"],
      ["Display, LED, pen", "320 $\\times$ 240 dashboard, state colour, floor trail",
       "live monitoring and evidence"]],
     "p{0.2\\linewidth}p{0.45\\linewidth}p{0.27\\linewidth}", True),
    ("sub", "Software Architecture"),
    ("p", "The controller (`intellibot_controller.py`) is a Webots supervisor written in Python 3.12 "
          "with NumPy [9] and OpenCV [8]. Only it and `perception.py` touch the Webots API; every "
          "algorithm (kinematics, PID, grid, A*, EKF, FSM, detector, docking, arm kinematics) is "
          "plain Python, so it can be unit-tested without the simulator. All constants live in "
          "`config.py`. Fig. 2 shows the data flow. The FSM is the only place that decides; "
          "perception and the EKF only report, and the governor can always overrule a speed "
          "command."),
    ("fig", "figures/fig2_architecture.png",
     "Control architecture: data flow in one 16 ms control step.", False),
    ("sub", "Kinematics and Motion Control"),
    ("p", "A body command $(v, \\omega)$ maps to wheel speeds by inverse kinematics [4]:"),
    ("eq", "\\omega_L = \\frac{v - \\omega L/2}{r}, \\qquad \\omega_R = \\frac{v + \\omega L/2}{r}",
     "eq:ik"),
    ("p", "and the encoder increments $\\Delta\\phi_L, \\Delta\\phi_R$ give the odometry update"),
    ("eq", "\\Delta s = \\frac{r\\,(\\Delta\\phi_R + \\Delta\\phi_L)}{2}, \\qquad "
           "\\Delta\\theta = \\frac{r\\,(\\Delta\\phi_R - \\Delta\\phi_L)}{L}", "eq:odom"),
    ("p", "The waypoint follower steers with a PID on heading error, with gains $k_p = 2.0$, "
          "$k_i = 0.05$ and $k_d = 0.15$ [10], turns in place when the error exceeds 0.6 rad, and slows over the "
          "last 0.5 m. It can drive forwards or backwards, whichever needs the smaller turn, "
          "because the robot has a camera at each end."),
    ("sub", "Perception"),
    ("ul", ["**Parcel detection.** Each camera frame goes BGR to HSV; masks for red, green and blue "
            "are cleaned with morphology, then blobs are filtered by area and aspect ratio. The hue "
            "windows avoid the yellow and cyan zone colours and the yellow decoy robot. A detection "
            "must persist for 2 frames.",
            "**Parcel position.** The pixel column gives a bearing; the LiDAR range in a "
            "$\\pm 3^\\circ$ window around that bearing gives the distance. The arm post blinds "
            "$\\pm 6^\\circ$ behind the LiDAR, so the robot first turns a rear-sector parcel out of "
            "that blind zone.",
            "**LiDAR.** Ray order and offset were measured in Webots (index increases clockwise, "
            "$+0.02$ rad offset). Returns closer than 0.15 m are the robot itself and are discarded.",
            "**Camera health.** Each frame is checked for BLANK and FROZEN (30 identical frames while "
            "moving) images. An unhealthy camera is ignored instead of trusted."]),
    ("fig", "figures/fig3_detector.png",
     "Detector output on a 320 $\\times$ 240 front-camera frame from Webots: P3 (blue) and P2 "
     "(green) boxed with their confidence; the red parcel at the edge is partly out of view.", False),
    ("fig", "figures/fig4_lidar.png",
     "LiDAR hits from a Webots run transformed into the world frame with the measured ray order "
     "and offset (360,554 points). Shelf edges, crates and walls line up with the map, which "
     "validated the calibration.", False),
    ("sub", "Localisation: Extended Kalman Filter"),
    ("p", "The state is the pose $\\mathbf{x} = [x, y, \\theta]^{\\mathsf T}$. Odometry drives the "
          "prediction [2]:"),
    ("eq", "\\hat{\\mathbf{x}}_{k|k-1} = \\hat{\\mathbf{x}}_{k-1} + "
           "\\begin{bmatrix} \\Delta s\\cos\\theta_{k-1} \\\\ \\Delta s\\sin\\theta_{k-1} \\\\ "
           "\\Delta\\theta \\end{bmatrix}", "eq:pred"),
    ("eq", "P_{k|k-1} = F_k P_{k-1} F_k^{\\mathsf T} + Q_k(\\Delta s, \\Delta\\theta)", "eq:cov"),
    ("p", "and GPS and IMU correct it with innovation "
          "$\\boldsymbol{\\nu}_k = \\mathbf{z}_k - H\\hat{\\mathbf{x}}_{k|k-1}$ and gain"),
    ("eq", "S_k = H P_{k|k-1} H^{\\mathsf T} + R, \\qquad K_k = P_{k|k-1} H^{\\mathsf T} S_k^{-1}",
     "eq:gain"),
    ("p", "Every correction is gated by its normalised innovation squared, "
          "$\\boldsymbol{\\nu}_k^{\\mathsf T} S_k^{-1} \\boldsymbol{\\nu}_k < \\gamma$, with "
          "$\\gamma = 9.21$ for GPS and 6.63 for the IMU ($\\chi^2$, 99 %) [7]. A sensor's $R$ is "
          "inflated as its health drops. If GPS is silent for 0.5 s the filter dead-reckons on "
          "odometry and IMU. If GPS fixes are rejected but agree with each other for 0.5 s, the "
          "robot was displaced (for example by a collision), so it relocalises and the planner is "
          "told. A matching re-anchor exists for the IMU heading. The navigation pose is the EKF "
          "output; ground truth is read only for evaluation."),
    ("sub", "Mapping, Planning and Replanning"),
    ("p", "The static map is a 0.1 m occupancy grid inflated by the robot radius plus a 7 cm margin "
          "(0.25 m). A* [3] (8-connected, Euclidean heuristic) finds a path, which is smoothed by "
          "line-of-sight shortcuts and resampled every 0.3 m. A dynamic layer marks cells from "
          "LiDAR hits at 10 Hz and clears them when rays pass through, so a moved obstacle "
          "disappears within four scans. The decision maker replans from the current pose when "
          "the next 1.5 m of route is blocked, the robot deviates more than 0.6 m, it is stuck for "
          "5 s, it bumps something or the EKF relocalises. For a moving obstacle it first waits up "
          "to 4 s before detouring."),
    ("sub", "Speed Governor"),
    ("p", "Every command passes a governor that caps speed so the robot can always stop before the "
          "nearest obstacle in its corridor:"),
    ("eq", "v_{\\mathrm{cap}} = \\min\\left(v_{\\mathrm{cruise}},\\, v_{\\mathrm{clear}},\\, "
           "v_{\\mathrm{carry}},\\, v_{\\mathrm{approach}},\\, v_{\\mathrm{health}}\\right)",
     "eq:vcap"),
    ("p", "where $v_{\\mathrm{clear}}$ is the largest speed whose stopping distance fits the "
          "measured clearance,"),
    ("eq", "d_{\\mathrm{stop}}(v) = v\\, t_{\\mathrm{react}} + \\frac{v^{2}}{2\\, a_{\\mathrm{brake}}} "
           "+ d_{\\mathrm{margin}}", "eq:dstop"),
    ("p", "Cruise is 0.70 m/s, 0.50 m/s while carrying, 0.12 m/s with degraded LiDAR or "
          "localisation, and zero if the LiDAR fails. Acceleration is limited to 1.0 m/s$^2$ and "
          "yaw acceleration to 9 rad/s$^2$; an emergency stop skips the ramp."),
    ("sub", "Mission State Machine"),
    ("p", "The decision maker is a finite-state machine (Fig. 5): ARM\\_STOW, FIND\\_PACKAGE, "
          "NAVIGATE, DOCK\\_PICK, ARM\\_PICK, VERIFY\\_ATTACHMENT, PLAN\\_DELIVERY, DELIVER, "
          "DOCK\\_PLACE, ARM\\_PLACE, VERIFY\\_DELIVERY and DONE, with ARM\\_RECOVER and ERROR for "
          "failures. Each state has a timeout and logs its reason on exit. In interactive mode the "
          "robot asks for the next task at the console and returns to the centre between missions. "
          "Every step ends on a sensor check: the grasp is verified by finger width and force pads, "
          "the placement by the palm range-finder. Rack docking and placing retry in the same way."),
    ("fig", "figures/fig5_fsm.png", "Mission state machine (`decision_maker.py`).", True),
    ("sub", "Manipulation and Docking"),
    ("p", "Docking fits a line to the parcel's or rack's front face in the LiDAR scan, drives to a "
          "pre-dock point 0.20 m back on that line, then creeps in until range, lateral and heading "
          "errors are under 8 mm, 15 mm and 1.5$^\\circ$. The arm then follows joint trajectories "
          "from planar inverse kinematics [5]: pre-grasp, grasp at floor level, close the fingers, "
          "and lock the Connector latch. A grasp is accepted only if the finger width and force "
          "pads agree with a 20 cm parcel. At the rack the robot picks a free slot (levels L1 or "
          "L2), docks, places, releases and verifies that the parcel stayed in the slot (Fig. 6)."),
    ("fig", "figures/fig6_arm.png",
     "Arm geometry from `config.py` (side view, robot frame): stow/carry pose, pre-grasp and floor "
     "grasp (left); placing on rack levels L1 and L2 (right). Level 3 is out of reach. The orange "
     "line is the LiDAR plane the arm must stay clear of while stowed.", True),
    ("sub", "Moving Obstacles"),
    ("p", "The carts and the decoy robot are moved kinematically by the supervisor, which writes "
          "their pose each step. An earlier design forced their velocity on physics bodies, which "
          "shoved the robot with unlimited force and sometimes froze them when the physics engine "
          "put the body to sleep. Each mover now runs its own small state machine (DRIVE, TURN, "
          "YIELD, BACKOFF, HOLD): it stops when IntelliBot is ahead within 0.45 m, backs up "
          "0.45 m if the robot has not passed after 4 s, then carries on. The decoy turns round at "
          "each lane end, spins its wheels and flashes its beacon while giving way. IntelliBot is "
          "told none of this; it sees an obstacle on its sensors."),
    ("sub", "Code Organisation and Testing"),
    ("p", "The controller package has about 40 modules. 312 pytest tests cover kinematics, PID, "
          "grid, A*, EKF, safety, detector, docking, arm kinematics, the FSM, the movers and world "
          "generation, and run without Webots in about 90 s. An offline kinematic simulator runs "
          "whole missions for ablation studies. In Webots, each run writes `run.csv` (10 Hz state, "
          "poses, ground truth, speed caps), `manip.csv` (arm and gripper), "
          "`mission_report.json` and a console log. Scenarios (normal runs and ablations) run in "
          "batch with `tools/run_scenario.py`."),

    ("sec", "Results and Discussion"),
    ("p", "The full mission works in Webots: P3 is found, picked, carried and placed on the Zone B "
          "rack in 72.9 s, 0.6 cm from the slot centre, with no contacts and 5 mm localisation "
          "error. Every number below states whether it comes from Webots or from the offline "
          "kinematic simulator."),
    ("sub", "Full Mission in Webots"),
    ("tab", "Full mission in Webots (P3 to Zone B, batch run)",
     ["Measure", "Value"],
     [["Result", "DELIVERED, verified by sensors and by ground-truth audit"],
      ["Mission time", "72.9 s simulated"],
      ["Placement error", "0.6 cm from slot centre"],
      ["Bumper contacts", "0"],
      ["EKF error vs ground truth", "5 mm position RMSE, 0.12$^\\circ$ heading RMSE"],
      ["Mean NEES", "0.37 (consistent: below 3 for a 3-state pose)"],
      ["Cruise speed", "0.70 m/s (0.50 m/s carrying)"]],
     "p{0.42\\linewidth}p{0.48\\linewidth}", False),
    ("p", "An interactive GUI run on the final build (decoy robot on the lane) delivered P1 to Zone A "
          "rack slot RA-L1-B1 in 93.6 s, with 17.5 m driven and one PATH\\_BLOCKED replan. The EKF "
          "tracked ground truth with 5.7 mm and 0.17$^\\circ$ RMSE (mean NEES 0.49, 1183 samples). "
          "There were no bumper contacts and no emergency stops; minimum corridor clearance was "
          "0.07 m. The decoy robot gave way twice; CART\\_2 gave way twice and backed up both "
          "times (Figs. 7 and 8)."),
    ("fig", "figures/fig7_delivery.png",
     "Webots, end of a P3-to-Zone-B delivery at $t$ = 72.4 s (top view). The yellow trail is the "
     "path driven. Top left: the on-robot dashboard (state VERIFY\\_DELIVERY, EKF $\\sigma$ 7 mm, "
     "all sensors OK, cruise cap 0.70 m/s, no bumps). Right: front and rear camera views.", False),
    ("fig", "figures/fig8_ui.png",
     "The Webots user interface during a mission (state DOCK\\_PICK, $t$ = 13.4 s): scene tree with "
     "its nodes (left), dashboard and camera "
     "overlays, and the console log of the LiDAR docking fit (bottom). Captured before the grey "
     "sphere obstacle was replaced by the yellow decoy robot.", True),
    ("sub", "Kinematics and Odometry"),
    ("p", "Commanded and measured motion agree within 1 % (Table IV), which validates the wheel "
          "model before anything is built on it."),
    ("tab", "Commanded vs measured motion in Webots",
     ["$v_{\\mathrm{cmd}}$ (m/s)", "$\\omega_{\\mathrm{cmd}}$ (rad/s)", "$v$ meas.",
      "$\\omega$ meas.", "Error"],
     [["0.20", "0.00", "0.1982", "0.0000", "$-$0.90 %"],
      ["0.00", "1.00", "0.0000", "0.9906", "$-$0.94 %"],
      ["0.20", "0.50", "0.1982", "0.4953", "$-$0.90 %, $-$0.94 %"],
      ["$-$0.10", "0.00", "$-$0.0991", "0.0000", "+0.90 %"]],
     "ccccc", False),
    ("p", "Pure odometry drifts 0.8\u20132.2 cm over 1\u20132 m and up to 4.7$^\\circ$ after a "
          "square path. That is small per metre but unbounded over a mission, which is why "
          "odometry only drives the EKF's prediction."),
    ("sub", "Path Tracking"),
    ("p", "On an early full mission run, the robot followed the A* reference with a mean "
          "cross-track error of 0.4 cm and a maximum of 6.2 cm (723 samples, 69 s). The largest "
          "errors occur at the turn into the pickup area, where the follower turns in place "
          "(Fig. 9)."),
    ("fig", "figures/fig9_path.png",
     "A* reference (dashed) and the driven path coloured by FSM state, from a Webots mission log "
     "(69 s, 723 samples). Orange cells are the 0.25 m inflation around obstacles.", False),
    ("sub", "Localisation: Why the EKF"),
    ("p", "The offline ablation (Fig. 10) compares three navigation poses on the same mission. "
          "Odometry alone drifts to 22 cm RMSE. Raw GPS is noisy and failed the mission at docking "
          "(it needs centimetre accuracy). The EKF stays at 1.1 cm and only reaches 1.7 cm through "
          "a 10 s GPS outage, because odometry and IMU carry it through. When the robot is "
          "displaced mid-route, relocalisation cuts the error from 15.9 cm to 5.2 cm RMSE and the "
          "mission time from 93.6 s to 73.4 s."),
    ("fig", "figures/fig10_ablation.png",
     "Offline ablation of the navigation pose source (`tools/run_experiments.py`).", True),
    ("sub", "Speed Governor and Dynamic Replanning"),
    ("p", "Offline, the governor made missions 3\u20134 % faster than a fixed 0.30 m/s (72.8 s vs "
          "76.2 s for P3 to B) while keeping the same clearance and zero contacts. With a box "
          "appearing on the route, continuous replanning detoured early (PATH\\_BLOCKED) and "
          "finished in 73.5 s with 0.20 m clearance; the legacy safety-stop approach got within "
          "0.10 m and took 82.2 s. A cart crossing at 0.4 m/s still caused contacts in both "
          "variants in that model, which is what led to the give-way behaviour of the movers in "
          "Webots."),
    ("sub", "Problems Found and Fixed"),
    ("p", "Testing exposed real bugs; each was traced to a root cause before it was fixed "
          "(Table V)."),
    ("tab", "Problems found during testing, root causes and fixes",
     ["Symptom", "Root cause", "Fix"],
     [["Webots aborted with 0x40000015 after a few minutes; NumPy MemoryError",
       "Webots R2025a leaks memory on every `Display.setFont()` call; the dashboard called it 3 times "
       "per refresh, so Webots grew $\\sim$60\u201380 MB/s to $\\sim$10 GB",
       "set the font once and paste the static map as one image; memory now plateaus at "
       "600\u2013735 MB"],
      ["27 rear-bumper hits, $-$0.17 m clearance, 21$^\\circ$ heading RMSE in one run",
       "a velocity-forced 0.75 m/s sphere rammed the waiting robot and a crate on every pass",
       "slower lane clear of the crate, kinematic movers that give way, decoy robot"],
      ["Moving obstacle froze mid-aisle; robot deadlocked behind it",
       "the physics engine put the commanded body to sleep",
       "kinematic movers (pose written every step)"],
      ["DOCK\\_PICK failed: parcel face not found by LiDAR",
       "the arm post blinds $\\pm6^\\circ$ behind the LiDAR; parcel range fell back to a bounding "
       "box 0.3 m short",
       "turn the parcel out of the rear blind sector before trusting range"],
      ["Robot froze silently", "stuck detection used the governed command, which was 0 while held",
       "stuck check uses the pre-governor command"],
      ["Camera health flapped to FROZEN", "identical frames while standing still counted as frozen",
       "count only while the base moves"]],
     "p{0.27\\linewidth}p{0.38\\linewidth}p{0.27\\linewidth}", True),
    ("p", "One more bug surfaced in the GUI run above. When the operator asked for P1 a second "
          "time, after it was already on the rack, the camera saw P1 on the rack. That estimate "
          "was more than 1.5 m from the pickup position, so the sanity check snapped it back to "
          "the empty pickup spot, and docking failed with PARCEL\\_FACE\\_NOT\\_FOUND\\_BY\\_LIDAR, "
          "ending the session in ERROR. The robot now checks its own rack record first and rejects "
          "the task (\u201cP1 is already on rack slot RA-L1-B1\u201d), returning to the task prompt."),
    ("sub", "Discussion"),
    ("p", "The EKF is the component that made everything else possible: docking needs centimetre "
          "poses, and only the fused estimate delivered them, even through a simulated GPS outage. "
          "Making the speed limit a function of clearance let us raise cruise speed from "
          "0.30 to 0.70 m/s without losing margin. The biggest lessons came from the simulator "
          "itself: two of the worst failures (the memory leak and the frozen obstacle) were Webots "
          "behaviours, not algorithm bugs, and were only found because every run wrote logs we "
          "could read afterwards. Results are still limited: the Webots numbers come from a few "
          "runs per configuration, the 10-run campaign predates the arm, and the Webots GPS is "
          "noise-free, which flatters localisation."),

    ("sec", "Conclusion"),
    ("p", "IntelliBot meets the project's objectives in simulation: it finds, picks, carries and "
          "places a parcel autonomously, through moving traffic, with centimetre-level accuracy "
          "and no contacts."),
    ("ul", ["**Closed loop, not scripted.** Every route comes from A* on the robot's own map, every "
            "parcel position from its cameras and LiDAR, and the parcel is physically held by the "
            "gripper.",
            "**Fused localisation.** The EKF kept position error at 5\u20136 mm in Webots and "
            "rode through a 10 s GPS outage offline, where it was 20 times more accurate than odometry "
            "alone.",
            "**Safe speed.** The clearance-based governor let cruise speed rise from 0.30 to "
            "0.70 m/s with zero contacts in the reported runs.",
            "**Engineering discipline.** One configuration generates both the world and the map, "
            "312 tests run without the simulator, and every run leaves logs that let us find root "
            "causes, including a simulator memory leak."]),
    ("p", "The main limits are honest ones: few Webots runs per configuration, a noise-free "
          "simulated GPS, and a single robot."),

    ("sec", "Future Work"),
    ("ol", ["**Statistical evaluation.** Run each scenario (normal and ablations) at least "
            "20 times in Webots and report success rates with confidence intervals.",
            "**LiDAR map matching or SLAM.** Localise against the shelves themselves, so the robot "
            "no longer depends on GPS, which does not exist indoors in a real warehouse.",
            "**Realistic GPS/indoor positioning noise.** Add bias, multipath and outages to the "
            "simulated GPS so the EKF is tested as it would be on hardware.",
            "**Multi-robot fleet.** Turn the decoy robot into a real second IntelliBot and use the "
            "existing fleet manager and traffic reservations to share aisles.",
            "**Learned perception.** Replace colour thresholds with a small object detector "
            "trained on simulator frames, so parcels need not have saturated colours.",
            "**Retrieve missions.** Pick from shelves back to a zone; this needs a higher-reach "
            "arm or a lift for level 3.",
            "**Hardware.** Port the plain-Python core to ROS 2 and a real differential-drive base "
            "with a low-cost arm."]),
]

REFERENCES = [
    "Cyberbotics Ltd., *Webots: Open-Source Robot Simulator*, version R2025a. [Online]. "
    "Available: https://cyberbotics.com/doc/reference/index",
    "S. Thrun, W. Burgard, and D. Fox, *Probabilistic Robotics*. Cambridge, MA, USA: MIT Press, 2005.",
    "P. E. Hart, N. J. Nilsson, and B. Raphael, \u201cA formal basis for the heuristic determination "
    "of minimum cost paths,\u201d *IEEE Trans. Syst. Sci. Cybern.*, vol. 4, no. 2, pp. 100\u2013107, 1968.",
    "R. Siegwart, I. R. Nourbakhsh, and D. Scaramuzza, *Introduction to Autonomous Mobile Robots*, "
    "2nd ed. Cambridge, MA, USA: MIT Press, 2011.",
    "J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 4th ed. London, U.K.: Pearson, 2017.",
    "D. Fox, W. Burgard, and S. Thrun, \u201cThe dynamic window approach to collision avoidance,\u201d "
    "*IEEE Robot. Autom. Mag.*, vol. 4, no. 1, pp. 23\u201333, 1997.",
    "Y. Bar-Shalom, X. R. Li, and T. Kirubarajan, *Estimation with Applications to Tracking and "
    "Navigation*. New York, NY, USA: Wiley, 2001.",
    "G. Bradski, \u201cThe OpenCV library,\u201d *Dr. Dobb's J. Softw. Tools*, 2000.",
    "C. R. Harris *et al.*, \u201cArray programming with NumPy,\u201d *Nature*, vol. 585, pp. 357\u2013362, 2020.",
    "K. J. \u00c5str\u00f6m and R. M. Murray, *Feedback Systems: An Introduction for Scientists and "
    "Engineers*, 2nd ed. Princeton, NJ, USA: Princeton Univ. Press, 2021.",
]

# In the two-column HTML/PDF every figure sits in one column; a two-panel figure
# is shown as its panels stacked so it stays legible at column width.
HTML_FIG_OVERRIDE = {"figures/fig6_arm.png": ["figures/fig6_arm_floor.png",
                                              "figures/fig6_arm_rack.png"]}

ROMAN = ["I", "II", "III", "IV", "V", "VI", "VII", "VIII", "IX", "X"]

# ------------------------------------------------------------------ inline parsing
TOKEN = re.compile(r"(\$[^$]+\$|`[^`]+`|\*\*[^*]+\*\*|\*[^*]+\*)")
TEX_ESC = {"&": r"\&", "%": r"\%", "#": r"\#", "~": r"\textasciitilde{}", "^": r"\^{}",
           "\u2013": "--", "\u2014": "---", "\u201c": "``", "\u201d": "''", "\u00c5": r"\AA{}",
           "\u00f6": r"\"o", "\u2212": "$-$"}


def tex_text(s):
    """Plain text -> LaTeX. Underscores are already written as \\_ in CONTENT."""
    out = []
    i = 0
    while i < len(s):
        c = s[i]
        if c == "\\" and i + 1 < len(s) and s[i + 1] == "_":
            out.append(r"\_")
            i += 2
            continue
        if c == "_":
            out.append(r"\_")
        elif c == "<":
            out.append(r"$<$")
        elif c == ">":
            out.append(r"$>$")
        else:
            out.append(TEX_ESC.get(c, c))
        i += 1
    return "".join(out)


def tex_inline(s):
    parts = TOKEN.split(s)
    out = []
    for p in parts:
        if not p:
            continue
        if p.startswith("$"):
            out.append(p)
        elif p.startswith("`"):
            out.append(r"\texttt{" + tex_text(p[1:-1]).replace(r"\_", r"\_\allowbreak{}") + "}")
        elif p.startswith("**"):
            out.append(r"\textbf{" + tex_inline(p[2:-2]) + "}")
        elif p.startswith("*"):
            out.append(r"\textit{" + tex_inline(p[1:-1]) + "}")
        else:
            out.append(tex_text(p))
    return "".join(out)


def html_inline(s):
    parts = TOKEN.split(s)
    out = []
    for p in parts:
        if not p:
            continue
        if p.startswith("$"):
            out.append('<span class="m">' + html.escape(p[1:-1]) + "</span>")
        elif p.startswith("`"):
            out.append("<code>" + html.escape(p[1:-1].replace("\\_", "_")) + "</code>")
        elif p.startswith("**"):
            out.append("<b>" + html_inline(p[2:-2]) + "</b>")
        elif p.startswith("*"):
            out.append("<i>" + html_inline(p[1:-1]) + "</i>")
        else:
            out.append(html.escape(p.replace("\\_", "_")))
    return "".join(out)


def cite_tex(s):
    """[1], [4], [10] -> \\cite{r1}, ... (only bracketed integers)."""
    return re.sub(r"\[(\d+)\]", lambda m: r"~\cite{r" + m.group(1) + "}", s).replace(" ~\\cite", "~\\cite")


# ------------------------------------------------------------------------- LaTeX
def build_tex():
    L = [r"\documentclass[conference]{IEEEtran}",
         r"\IEEEoverridecommandlockouts",
         r"\usepackage{amsmath,amssymb,amsfonts}",
         r"\usepackage{graphicx}",
         r"\usepackage{array}",
         r"\usepackage{url}",
         r"\usepackage[T1]{fontenc}",
         r"\usepackage{textcomp}",
         r"\begin{document}",
         r"\title{" + tex_text(TITLE) + r"\\[4pt] {\normalsize Code: \texttt{" + REPO + "}}}",
         r"\author{" + "\n\\and\n".join(
             r"\IEEEauthorblockN{" + a + r"}\IEEEauthorblockA{\textit{" + AFFILIATION + "}}"
             for a in AUTHORS) + "}",
         r"\maketitle",
         r"\begin{abstract}", ABSTRACT.replace("%", r"\%"), r"\end{abstract}",
         r"\begin{IEEEkeywords}", KEYWORDS, r"\end{IEEEkeywords}", ""]
    eq_labels = {}
    for kind, *a in CONTENT:
        if kind == "sec":
            L.append(r"\section{" + tex_text(a[0]) + "}")
        elif kind == "sub":
            L.append(r"\subsection{" + tex_text(a[0]) + "}")
        elif kind == "p":
            L.append(cite_tex(tex_inline(a[0])) + "\n")
        elif kind in ("ul", "ol"):
            env = "itemize" if kind == "ul" else "enumerate"
            L.append(r"\begin{" + env + "}")
            L += [r"\item " + cite_tex(tex_inline(it)) for it in a[0]]
            L.append(r"\end{" + env + "}")
        elif kind == "eq":
            L += [r"\begin{equation}", a[0], r"\label{" + a[1] + "}", r"\end{equation}"]
        elif kind == "fig":
            f, cap, wide = a
            env = "figure*" if wide else "figure"
            width = r"0.9\textwidth" if wide else r"\columnwidth"
            L += [r"\begin{" + env + "}[!t]", r"\centering",
                  r"\includegraphics[width=" + width + "]{" + f + "}",
                  r"\caption{" + tex_inline(cap) + "}", r"\end{" + env + "}"]
        elif kind == "tab":
            cap, head, rows, spec, wide = a
            env = "table*" if wide else "table"
            L += [r"\begin{" + env + "}[!t]", r"\caption{" + tex_text(cap) + "}",
                  r"\centering", r"\footnotesize", r"\renewcommand{\arraystretch}{1.2}",
                  r"\begin{tabular}{" + spec + "}", r"\hline",
                  " & ".join(r"\textbf{" + tex_inline(h) + "}" for h in head) + r" \\ \hline"]
            L += [" & ".join(tex_inline(c) for c in r) + r" \\" for r in rows]
            L += [r"\hline", r"\end{tabular}", r"\end{" + env + "}"]
    L.append(r"\begin{thebibliography}{00}")
    for i, ref in enumerate(REFERENCES, 1):
        ref_t = re.sub(r"(https?://\S+)", lambda m: r"\url{" + m.group(1) + "}", ref)
        parts = re.split(r"(\\url\{[^}]*\})", ref_t)
        ref_t = "".join(p if p.startswith(r"\url") else tex_inline(p) for p in parts)
        L.append(r"\bibitem{r" + str(i) + "} " + ref_t)
    L += [r"\end{thebibliography}", r"\end{document}", ""]
    return "\n".join(L)


# -------------------------------------------------------------------------- HTML
CSS = r"""
@page { size: letter; margin: 0.75in 0.625in 1in 0.625in; }
* { box-sizing: border-box; }
html, body { background: #fff; color: #000; margin: 0; }
body { font-family: "Times New Roman", Times, serif; font-size: 10pt; line-height: 1.18; }
.title { text-align: center; font-size: 24pt; line-height: 1.15; margin: 0 0 14pt; font-weight: normal; }
.repo { text-align: center; font-size: 10pt; margin: -6pt 0 12pt; }
.repo a { color: #000; text-decoration: none; font-family: "Courier New", monospace; }
.authors { display: flex; justify-content: center; gap: 0.6in; text-align: center; margin-bottom: 18pt; }
.authors .n { font-size: 11pt; } .authors .a { font-size: 10pt; font-style: italic; }
.cols { column-count: 2; column-gap: 0.25in; text-align: justify; hyphens: auto; }
.abstract, .keywords { font-weight: bold; font-size: 9pt; margin: 0 0 8pt; text-indent: 1em; }
.abstract i, .keywords i { font-weight: bold; }
h2 { font-size: 10pt; font-variant: small-caps; font-weight: normal; text-align: center; margin: 10pt 0 5pt; break-after: avoid; }
h3 { font-size: 10pt; font-style: italic; font-weight: normal; margin: 7pt 0 3pt; break-after: avoid; }
p { margin: 0; text-indent: 1em; }
h2 + p, h3 + p, .eq + p.cont { text-indent: 0; }
p.cont { text-indent: 0; }
p.rag { text-align: left; }
ul, ol { margin: 3pt 0 3pt 0; padding-left: 1.6em; }
li { margin-bottom: 2pt; }
code { font-family: "Courier New", monospace; font-size: 8.6pt; overflow-wrap: anywhere; }
.eq { display: flex; align-items: center; margin: 5pt 0; break-inside: avoid; }
.eq .body { flex: 1; text-align: center; overflow: hidden; }
.eq .num { width: 2.2em; text-align: right; }
figure { margin: 8pt 0; break-inside: avoid; text-align: center; }
figure img { width: 100%; height: auto; display: block; margin: 0 auto 3pt; }
figure.wide { column-span: all; margin: 10pt 0; }
figure.wide img { width: 88%; }
figcaption { font-size: 8pt; text-align: left; margin-top: 4pt; }
.tbl { margin: 8pt 0; break-inside: avoid; }
.tbl.wide { column-span: all; }
.tcap { font-size: 8pt; text-align: center; font-variant: small-caps; margin-bottom: 4pt; }
.tcap .tn { display: block; font-variant: normal; }
table { border-collapse: collapse; width: 100%; font-size: 8pt; line-height: 1.15; }
th, td { border-top: 0.5pt solid #000; border-bottom: 0.5pt solid #000; padding: 2.5pt 4pt; text-align: left; vertical-align: top; }
th { font-weight: bold; }
td.c, th.c { text-align: center; }
.refs { font-size: 8pt; padding-left: 0; list-style: none; counter-reset: r; }
.refs li { position: relative; padding-left: 2.6em; margin-bottom: 2pt; text-indent: 0; text-align: left; }
.refs li::before { counter-increment: r; content: "[" counter(r) "]"; position: absolute; left: 0; }
.katex { font-size: 1.05em; }
.katex-display { margin: 0; }
.eq { margin: 4pt 0; }
"""


def build_html():
    eq_no = 0
    fig_no = 0
    tab_no = 0
    sec_no = 0
    sub_no = 0
    out = []
    prev = None
    for kind, *a in CONTENT:
        if kind == "sec":
            sec_no += 1
            sub_no = 0
            out.append(f"<h2>{ROMAN[sec_no - 1]}. {html.escape(a[0])}</h2>")
        elif kind == "sub":
            sub_no += 1
            out.append(f"<h3>{chr(64 + sub_no)}. {html.escape(a[0])}</h3>")
        elif kind == "p":
            classes = []
            if prev == "eq":
                classes.append("cont")
            if a[0].count("\\_") >= 4:        # many unbreakable STATE_NAMES
                classes.append("rag")
            cls = f' class="{" ".join(classes)}"' if classes else ""
            out.append(f"<p{cls}>{html_inline(a[0])}</p>")
        elif kind in ("ul", "ol"):
            tag = kind
            out.append(f"<{tag}>" + "".join(f"<li>{html_inline(it)}</li>" for it in a[0]) + f"</{tag}>")
        elif kind == "eq":
            eq_no += 1
            out.append(f'<div class="eq"><div class="body"><span class="md">{html.escape(a[0])}</span>'
                       f'</div><div class="num">({eq_no})</div></div>')
        elif kind == "fig":
            fig_no += 1
            f, cap, wide = a
            imgs = "".join(f'<img src="{x}">' for x in HTML_FIG_OVERRIDE.get(f, [f]))
            out.append(f'<figure>{imgs}<figcaption>Fig. {fig_no}. '
                       f"{html_inline(cap)}</figcaption></figure>")
        elif kind == "tab":
            tab_no += 1
            cap, head, rows, spec, wide = a
            centre = spec.replace(" ", "") == "c" * len(head)
            tdc = ' class="c"' if centre else ""
            widths = re.findall(r"p\{([\d.]+)\\linewidth\}", spec)
            cols = ("<colgroup>" + "".join(f'<col style="width:{float(w) / sum(map(float, widths)) * 100:.1f}%">'
                                           for w in widths) + "</colgroup>") if widths else ""
            t = [f'<div class="tbl"><div class="tcap">'
                 f'<span class="tn">TABLE {ROMAN[tab_no - 1]}</span>{html_inline(cap)}</div><table>{cols}'
                 "<tr>" + "".join(f"<th{tdc}>{html_inline(h)}</th>" for h in head) + "</tr>"]
            t += ["<tr>" + "".join(f"<td{tdc}>{html_inline(c)}</td>" for c in r) + "</tr>" for r in rows]
            t.append("</table></div>")
            out.append("".join(t))
        prev = kind
    refs = "".join(f"<li>{html_inline(r)}</li>" for r in REFERENCES)
    authors = "".join(f'<div><div class="n">{a}</div><div class="a">{AFFILIATION}</div></div>'
                      for a in AUTHORS)
    abstract = html_inline(ABSTRACT)
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<title>{html.escape(TITLE)}</title>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/KaTeX/0.16.9/katex.min.css">
<script src="https://cdnjs.cloudflare.com/ajax/libs/KaTeX/0.16.9/katex.min.js"></script>
<style>{CSS}</style></head>
<body>
<h1 class="title">{html.escape(TITLE)}</h1>
<div class="repo">Code: <a href="{REPO}">{REPO}</a></div>
<div class="authors">{authors}</div>
<div class="cols">
<p class="abstract"><i>Abstract</i>&mdash;{abstract}</p>
<p class="keywords"><i>Index Terms</i>&mdash;{html.escape(KEYWORDS)}</p>
{"".join(out)}
<h2>References</h2>
<ol class="refs">{refs}</ol>
</div>
<script>
document.querySelectorAll('.m').forEach(e => katex.render(e.textContent, e, {{throwOnError: true}}));
document.querySelectorAll('.md').forEach(e => katex.render(e.textContent, e, {{throwOnError: true, displayMode: true}}));
document.body.setAttribute('data-math', 'done');
</script>
</body></html>"""


def find_browser():
    for p in (r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
              r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
              r"C:\Program Files\Google\Chrome\Application\chrome.exe",
              shutil.which("chromium") or "", shutil.which("google-chrome") or ""):
        if p and os.path.exists(p):
            return p
    return None


def main():
    tex = build_tex()
    with open(os.path.join(HERE, "paper.tex"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write(tex)
    page = build_html()
    html_path = os.path.join(HERE, "paper.html")
    with open(html_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(page)
    print("wrote paper.tex and paper.html")
    browser = find_browser()
    if browser is None:
        print("no Edge/Chrome found: open paper.html and print to PDF, or compile paper.tex")
        return 0
    pdf = os.path.join(HERE, "paper.pdf")
    url = "file:///" + html_path.replace("\\", "/")
    cmd = [browser, "--headless", "--disable-gpu", "--no-pdf-header-footer",
           "--virtual-time-budget=15000", "--run-all-compositor-stages-before-draw",
           f"--print-to-pdf={pdf}", url]
    subprocess.run(cmd, check=False, timeout=120,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("wrote paper.pdf" if os.path.exists(pdf) else "PDF printing failed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
