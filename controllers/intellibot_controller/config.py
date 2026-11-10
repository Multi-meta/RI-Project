"""config.py — single source of truth for ALL constants.

Every module imports its numbers from here. No magic numbers anywhere else.
Units: meters, radians, seconds (convert to degrees only when printing).

Sections:
    1. Arena / world layout
    2. Robot physical parameters
    3. Device names
    4. Grid / mapping
    5. Planning (A*)
    6. Control (waypoint follower, PID)
    7. Perception (LiDAR, camera, detector)
    8. Decision maker (FSM)
    9. Task & mission
    10. Final-eval hooks (placeholders, do not use yet)
"""

import math

# ---------------------------------------------------------------- 1. Arena ---
ARENA_X_MIN, ARENA_X_MAX = -4.0, 4.0          # 8 m wide  (x east)
ARENA_Y_MIN, ARENA_Y_MAX = -3.0, 3.0          # 6 m deep  (y north)
WALL_THICKNESS = 0.1
WALL_HEIGHT = 0.3
BASIC_TIME_STEP_MS = 16                        # WorldInfo.basicTimeStep

# Shelves: (center_x, center_y, size_x, size_y, height, DEF name)
SHELVES = [
    (-2.0, 1.9, 1.6, 0.5, 1.0, "SHELF_1"),
    (2.0, 1.9, 1.6, 0.5, 1.0, "SHELF_2"),
    (-2.0, -1.9, 1.6, 0.5, 1.0, "SHELF_3"),
    (2.0, -1.9, 1.6, 0.5, 1.0, "SHELF_4"),
]
# Static crates: (center_x, center_y, size_x, size_y, height, DEF name)
CRATES = [
    (0.0, 0.6, 0.4, 0.4, 0.4, "OBST_1"),
    (-0.8, -0.9, 0.4, 0.4, 0.4, "OBST_2"),
]

# Small extra obstacles with DIFFERENT SHAPES so the robot has to steer
# around more than axis-aligned boxes. Cylinders/spheres are entered into the
# occupancy grid as their bounding squares (2r x 2r) — conservative and safe.
# Positions chosen to keep every aisle >= 1.0 m and BOTH routes to Zone B
# (asserted by tests/test_warehouse.py):
#   * CYL_OBST_1  (1.10,  0.70) r=0.15 barrel in the NE center aisle
#   * SPH_OBST_1  (-1.00, 0.90) r=0.15 ball north of the west lane
#   * OBST_3      (-1.70, 0.60) 0.3 m crate north of the pickup approach
#   * CYL_OBST_2  (1.80, -0.15) r=0.12 slim pole forcing a weave to Zone B
# All sit in the central band (y in [-0.5, 1.2]); the outer corridors
# (y < -1.65 and y > 2.15) stay clear for the alternate-route test.
CYLINDERS = [                                  # (cx, cy, radius, height, DEF)
    (1.10, 0.70, 0.15, 0.35, "CYL_OBST_1"),
    (1.80, -0.15, 0.12, 0.25, "CYL_OBST_2"),
]
SPHERES = [                                    # (cx, cy, radius, DEF)
    (-1.00, 0.90, 0.15, "SPH_OBST_1"),
]
CRATES_EXTRA = [                               # (cx, cy, size, height, DEF)
    (-1.70, 0.60, 0.3, 0.3, "OBST_3"),
]

# STATIC_OBSTACLES: walls + shelves + crates, as (cx, cy, sx, sy) — the known
# map for the occupancy grid. Walls added programmatically below so the grid
# and the generated world always agree.
_ARENA_W = ARENA_X_MAX - ARENA_X_MIN
_ARENA_H = ARENA_Y_MAX - ARENA_Y_MIN
_WALLS = [
    (0.0, ARENA_Y_MAX + WALL_THICKNESS / 2, _ARENA_W + 2 * WALL_THICKNESS, WALL_THICKNESS),
    (0.0, ARENA_Y_MIN - WALL_THICKNESS / 2, _ARENA_W + 2 * WALL_THICKNESS, WALL_THICKNESS),
    (ARENA_X_MAX + WALL_THICKNESS / 2, 0.0, WALL_THICKNESS, _ARENA_H),
    (ARENA_X_MIN - WALL_THICKNESS / 2, 0.0, WALL_THICKNESS, _ARENA_H),
]
STATIC_OBSTACLES = (
    [(x, y, sx, sy) for (x, y, sx, sy, _h, _d) in SHELVES]
    + [(x, y, sx, sy) for (x, y, sx, sy, _h, _d) in CRATES]
    + [(x, y, 2 * r, 2 * r) for (x, y, r, _h, _d) in CYLINDERS]
    + [(x, y, 2 * r, 2 * r) for (x, y, r, _d) in SPHERES]
    + [(x, y, sz, sz) for (x, y, sz, _h, _d) in CRATES_EXTRA]
    + _WALLS
)

# Floor patches and small objects (world frame, object centers)
PICKUP_AREA = (-3.0, 0.0, 1.2, 1.6)           # (cx, cy, sx, sy), light gray patch
ZONE_A = (3.0, 0.9, 1.0, 1.0)                 # yellow patch
ZONE_B = (3.0, -0.9, 1.0, 1.0)                # cyan patch
ZONE_A_CENTER = (ZONE_A[0], ZONE_A[1])
ZONE_B_CENTER = (ZONE_B[0], ZONE_B[1])

# Packages: 0.2 m cubes, pure saturated colors. (DEF name, x, y, color)
PKG_SIZE = 0.2
PACKAGES = [
    ("PKG_P1", -3.1, 0.5, "red"),
    ("PKG_P2", -3.1, 0.0, "green"),
    ("PKG_P3", -3.1, -0.5, "blue"),
]

# Destination shelving (delivery racks) -------------------------------------
# One warehouse-style rack per dispatch zone, standing against the EAST wall
# directly behind its zone patch. The robot parks inside the zone, faces the
# rack (+x) and the arm places the parcel on a shelf level. Every derived
# number (slot centres, footprints for the occupancy grid, docking lines) is
# computed in shelving.py from these constants, so the world generator, the
# planner's map and the FSM always agree.
PKG_MASS = 0.5                                # kg, parcel mass (world + arm load)
DEST_RACK_DEPTH = 0.32                        # m, along x (front at x = front_x)
DEST_RACK_BAY_WIDTH = 0.45                    # m, along y, per bay (> open gripper 0.29 m)
DEST_RACK_BAYS = 2                            # bays per rack
DEST_RACK_LEVEL_TOPS = (0.15, 0.41, 0.67)     # m, top surface of each platform (L1..L3)
DEST_RACK_REACHABLE_LEVELS = (1, 2)           # levels the arm can serve (L3 = overflow)
DEST_RACK_PLATFORM_THICKNESS = 0.02           # m
DEST_RACK_POST_SIZE = 0.03                    # m, square uprights
DEST_RACK_PANEL_THICKNESS = 0.01              # m, back panel + kick plate
DEST_RACK_WALL_GAP = 0.02                     # m, gap between rack back and the wall
DEST_RACK_SLOT_INSET = 0.03                   # m, parcel near face behind the rack front
DEST_RACK_FRONT_X = (ARENA_X_MAX - DEST_RACK_WALL_GAP - DEST_RACK_DEPTH)   # 3.66 m
DEST_RACK_FRAME_COLOR = (0.40, 0.42, 0.46)    # steel grey: saturation ~0.13, outside every
DEST_RACK_SHELF_COLOR = (0.62, 0.63, 0.65)    #   detector HSV window (S_min = 80/255)
# zone -> (rack centre y, DEF name, slot-id prefix)
DEST_RACKS = {
    "A": (0.9, "DEST_RACK_A", "RA"),
    "B": (-0.9, "DEST_RACK_B", "RB"),
}
DEST_RACK_WIDTH = DEST_RACK_BAYS * DEST_RACK_BAY_WIDTH + DEST_RACK_POST_SIZE
DEST_RACK_CENTER_X = DEST_RACK_FRONT_X + DEST_RACK_DEPTH / 2.0
# Footprints (cx, cy, sx, sy) added to the static map below: the planner must
# never treat solid shelf structure as free space.
DEST_RACK_FOOTPRINTS = [(DEST_RACK_CENTER_X, cy, DEST_RACK_DEPTH, DEST_RACK_WIDTH)
                        for (cy, _d, _p) in DEST_RACKS.values()]
STATIC_OBSTACLES = STATIC_OBSTACLES + DEST_RACK_FOOTPRINTS

ROBOT_START = (0.0, -0.3, 0.0)                # (x, y, yaw)

# Colors for the world generator + detector association (BGR tuples live in the
# generator; here only the semantic mapping).
PACKAGE_COLORS = {"P1": "red", "P2": "green", "P3": "blue"}
ZONE_OF = {"A": ZONE_A_CENTER, "B": ZONE_B_CENTER}
COLOR_ZONE = {"A": "yellow", "B": "cyan"}

# ---------------------------------------------------------------- 2. Robot ---
BODY_LENGTH = 0.30                            # x (forward, FLU)
BODY_WIDTH = 0.20                             # y (left)
BODY_HEIGHT = 0.10                            # z (up)
BODY_MASS = 2.0
WHEEL_RADIUS = 0.05
WHEEL_SEPARATION = 0.24                       # between wheel centers
WHEEL_WIDTH = 0.04
MAX_WHEEL_SPEED = 12.0                        # rad/s (motor limit)
CASTER_RADIUS = 0.03
ROBOT_RADIUS = 0.18                           # circumscribed radius for planning
SAFETY_MARGIN = 0.07
INFLATION_RADIUS = ROBOT_RADIUS + SAFETY_MARGIN   # 0.25 m

# ---------------------------------------------------------------- 3. Devices ---
LIDAR_NAME = "lidar"
CAMERA_NAME = "camera"
GPS_NAME = "gps"
IMU_NAME = "imu"
LEFT_MOTOR = "left_motor"
RIGHT_MOTOR = "right_motor"
LEFT_ENCODER = "left_encoder"
RIGHT_ENCODER = "right_encoder"
# Final-eval devices (placeholders)
BUMPER_NAME = "bumper"
DYN_OBSTACLE_DEF = "DYN_OBSTACLE"
DYN_OBSTACLE_PARK = (3.5, 2.6)                # free NE corner, off every route

DEVICE_NAMES = {
    "lidar": LIDAR_NAME, "camera": CAMERA_NAME, "gps": GPS_NAME, "imu": IMU_NAME,
    "left_motor": LEFT_MOTOR, "right_motor": RIGHT_MOTOR,
    "left_encoder": LEFT_ENCODER, "right_encoder": RIGHT_ENCODER,
}

# ---------------------------------------------------------------- 4. Grid ---
GRID_RES = 0.1                                # m per cell
GRID_MIN_RADIUS = 0.05                        # lidar min range (validity)

# ---------------------------------------------------------------- 5. Planning ---
ASTAR_TIE_EPS = 1e-6                          # heuristic tie-break weight

# ---------------------------------------------------------------- 6. Control ---
V_MAX = 0.30                                  # m/s cruise limit
W_MAX = 1.5                                   # rad/s turn limit
WP_TOLERANCE = 0.10                           # m, intermediate waypoint
GOAL_TOLERANCE = 0.15                         # m, final waypoint
SLOWDOWN_DIST = 0.50                          # m, begin slowing near waypoint
TURN_IN_PLACE_THRESH = 0.6                    # rad, rotate first if worse
STOP_DIST = 0.30                              # m, LiDAR safety stop
FRONT_CONE = math.radians(30.0)               # ± cone for safety stop

# PID heading gains (tuned in Phase 5; documented in docs/progress_log.md)
HEADING_PID = {"kp": 2.0, "ki": 0.05, "kd": 0.15}

# ---------------------------------------------------------------- 7. Perception ---
# LiDAR
LIDAR_RAYS = 360
LIDAR_FOV = 2.0 * math.pi
LIDAR_MIN_RANGE = 0.05
LIDAR_MAX_RANGE = 4.0
LIDAR_MOUNT_Z = 0.14
# Empirically MEASURED via DRIVE_TEST_MODE=lidar + tools/calibrate_lidar.py
# (2026-10-07, 1321 scans vs known map, median |err| 8 cm): the ray index
# increases CLOCKWISE, i.e. mirrored vs the naive assumption, with a small
# +0.02 rad offset. This was the cause of the 0.6 m package-position error.
LIDAR_ANGLE_SIGN = -1.0                       # measured: index increases CW
LIDAR_ANGLE_OFFSET = 0.02                     # rad, from calibration
# Self-filter: the arm's rear support post crosses the LiDAR plane ~0.11 m
# behind the sensor. Webots LiDARs DO see the robot's own geometry (minRange
# only hides it, "but still occlude[s] other objects"), so returns closer than
# this radius are the robot itself and are discarded (-> inf). Nothing
# external can be closer than the bumpers (0.17 m), so no real obstacle is lost.
LIDAR_SELF_FILTER_RANGE = 0.15                # m

# Camera
# Both cameras use a PLANAR projection. Webots limits a planar camera's
# fieldOfView to [0, pi]; the previous value 3.14159265 (= pi, labelled
# "90 deg") collapsed the frustum (focal length ~ 0 px) so every frame was a
# smear of the colour straight ahead. 1.0 rad (57 deg) is the value the
# detector evaluation set, position-estimate evidence and the 10/10 campaign
# were captured with (docs/evidence/), so it is restored for BOTH cameras.
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_FOV = 1.0                              # rad, front camera (planar, < pi)
REAR_CAMERA_FOV = 1.0                         # rad, rear camera (planar, < pi)
CAMERA_PROJECTION = "planar"                  # Webots Camera.projection
CAMERA_NEAR = 0.01                            # m, Webots Camera.near
YIELD_WAIT_TIME = 8.0                         # s, pause for moving obstacle to pass
# Lens positions in the robot frame (FLU). The front lens sits on the body's
# front face, the rear lens on the back face, both at the validated mid-eval
# height (below the LiDAR plane and below the arm workspace, so the arm never
# occludes them while stowed). The generator writes EXACTLY these numbers.
CAMERA_MOUNT_X = 0.15                         # m, front lens (= BODY_LENGTH / 2)
CAMERA_MOUNT_Z = 0.12                         # m, both lenses
REAR_CAMERA_MOUNT_X = -0.15                   # m, rear lens (= -BODY_LENGTH / 2)
REAR_CAMERA_YAW = math.pi                     # rad, rear camera looks along -x
CAMERA_HOUSING_SIZE = (0.016, 0.04, 0.024)    # m (depth, width, height), BEHIND the lens
REAR_CAMERA_NAME = "rear_camera"
CAMERA_BLANK_STD_MIN = 2.0                    # grey-level std below this = blank frame
CAMERA_FROZEN_FRAMES = 30                     # identical consecutive frames = frozen
CAMERA_NOMINAL_MEAN = 90.0                    # grey level of a normally lit frame
CAMERA_DARK_MEAN = 15.0                       # below: camera treated as blind (lights off)

# Detector (OpenCV hue 0-180 scale). Kept out of the yellow(~30)/cyan(~90)
# hue bands so zone patches can never be confused with packages.
HSV_RANGES = {
    "red":   [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    "green": [((40, 80, 60), (80, 255, 255))],
    "blue":  [((105, 80, 60), (135, 255, 255))],
}
# Restored to the values validated by docs/evidence/detector_eval.md (they
# had been loosened to 30 px^2 / 0.4-2.5 to compensate for the degenerate
# pi-rad FOV, which also broke tests/test_detector.py::test_small_blob_rejected).
MIN_CONTOUR_AREA = 100.0                      # px^2 at 320x240 (P3 at ~4 m is ~14 px wide)
ASPECT_MIN, ASPECT_MAX = 0.6, 1.6             # bbox aspect ratio window

# Package position estimate
PKG_BEARING_WINDOW = math.radians(3.0)        # lidar min-range window around bearing

# ---------------------------------------------------------------- 8. FSM ---
FIND_ROT_SPEED = 0.6                          # rad/s scan rotation
FIND_MAX_TURNS = 2.0                          # full revolutions before timeout
DETECTION_CONSEC = 2                          # consecutive frames to accept
APPROACH_DISTANCE = 0.45                      # m, stand-off from package
PICK_PAUSE_S = 1.0                            # s, stop duration in PICK

# ---------------------------------------------------------------- 9. Task ---
TASK_PACKAGE = "P3"
TASK_PACKAGE_DEF = "PKG_P3"
TASK_PACKAGE_COLOR = "blue"
TASK_ZONE = "B"
AUTO_QUIT = True                              # simulationQuit(0) at DONE
MISSION_TIMEOUT_S = 600.0                     # s, non-interactive runs (was 300 s; docking +
                                              # pick + place adds ~1-2 min of sim time)

# Interactive multi-pickup loop:
#   True  -> after each delivery the robot returns to center and asks for the
#            next color/zone via console (simulation never auto-quits).
#   False -> original single-mission behaviour (DONE -> optional simulationQuit).
INTERACTIVE_MODE = True

# Max tolerated error (m) between camera+LiDAR package position estimate and
# the config ground-truth before falling back to the config position.
# A LiDAR ray hitting a static obstacle between robot and package can place
# the estimate 0.5-2.0 m off.  1.5 m is a safe threshold for the warehouse.
MAX_PKG_ESTIMATE_ERR = 1.5

# ------------------------------------------------------- 11. Viewpoints ---
def _look_at(eye, target):
    """Webots Viewpoint (position, axis-angle orientation) looking from eye to
    target with world +z up. Viewpoint frame: x = view direction, z = up."""
    fx, fy, fz = (t - e for t, e in zip(target, eye))
    n = math.sqrt(fx * fx + fy * fy + fz * fz)
    f = (fx / n, fy / n, fz / n)
    up = (0.0, 0.0, 1.0) if abs(f[2]) < 0.999 else (0.0, 1.0, 0.0)
    lx, ly, lz = (up[1] * f[2] - up[2] * f[1], up[2] * f[0] - up[0] * f[2],
                  up[0] * f[1] - up[1] * f[0])               # left = up x f
    n = math.sqrt(lx * lx + ly * ly + lz * lz)
    lft = (lx / n, ly / n, lz / n)
    u = (f[1] * lft[2] - f[2] * lft[1], f[2] * lft[0] - f[0] * lft[2],
         f[0] * lft[1] - f[1] * lft[0])                      # up = f x left
    # rotation matrix columns = (f, left, up) -> axis-angle
    m = [[f[0], lft[0], u[0]], [f[1], lft[1], u[1]], [f[2], lft[2], u[2]]]
    ang = math.acos(max(-1.0, min(1.0, (m[0][0] + m[1][1] + m[2][2] - 1) / 2)))
    if ang < 1e-9:
        axis = (0.0, 0.0, 1.0)
    elif math.pi - ang < 1e-6:                               # 180 deg case
        d = [math.sqrt(max(0.0, (m[i][i] + 1) / 2)) for i in range(3)]
        i = d.index(max(d))
        axis = [m[i][j] / (2 * d[i]) if j != i else d[i] for j in range(3)]
        axis = tuple(axis)
    else:
        s = 2 * math.sin(ang)
        axis = ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s)
    pos = " ".join(f"{v:.4f}" for v in eye)
    ori = " ".join(f"{v:.4f}" for v in axis) + f" {ang:.4f}"
    return pos, ori


# shifted right of center so the camera overlay (top-left) covers empty floor
VIEW_TOPDOWN = _look_at((-1.0, -0.001, 14.5), (-1.0, 0.0, 0.0))
VIEW_ANGLED = _look_at((-2.5, -7.0, 8.5), (0.3, 0.2, 0.0))

# ------------------------------------------------------- 10. Final-eval hooks ---
PICK_DISTANCE_THRESHOLD = 0.18                # final: attach distance
DYNAMIC_OBSTACLE_TRIGGER_TIME = -1.0          # s; <0 = disabled
WORLD_CHECK_ENABLED = True                    # supervisor world/config check
WORLD_CHECK_TOL = 0.01                        # m

# --------------------------------------- 11. P2 extras: dashboard, LED -------
DISPLAY_ENABLED = True                        # in-sim Display dashboard (6.10)
DISPLAY_NAME = "dashboard"
DISPLAY_WIDTH = 320
DISPLAY_HEIGHT = 200
LED_NAME = "status_led"                       # FSM state color indicator (6.11)
PEN_NAME = "pen"                              # floor trail (Phase 3.5)
PEN_ENABLED = True                            # draw the driven path on the floor
STATE_LED = {                                 # 0xRRGGBB per FSM state
    "IDLE": 0x888888,
    "FIND_PACKAGE": 0x0000FF,
    "NAVIGATE": 0x00A5FF,
    "PICK": 0xFFA500,
    "PLAN_DELIVERY": 0xFFFF00,
    "DELIVER": 0x00FF88,
    "DONE": 0x00FF00,
    "ERROR": 0xFF0000,
    "RETURN_TO_CENTER": 0x00FFFF,   # cyan
    "AWAIT_INPUT": 0xFF00FF,        # magenta
    # manipulation states (Phase 16 FSM v2 / 17.2 LED = FSM state)
    "ARM_STOW": 0x8888FF,
    "DOCK_PICK": 0x0088FF,
    "ARM_PICK": 0xFF8800,
    "VERIFY_ATTACHMENT": 0xFFCC00,
    "DOCK_PLACE": 0x00CC88,
    "ARM_PLACE": 0xFF6600,
    "VERIFY_DELIVERY": 0x88FF00,
    "ARM_RECOVER": 0xFF00AA,
}

# ------------------------------ 12. Repeatability campaign (6.8) -------------
# Start poses (x, y, yaw) for the N>=10 runs campaign. RUN_INDEX env var
# selects one; the supervisor teleports the robot there at startup and the
# run logs to logs/run_<index>.csv.
CAMPAIGN_STARTS = [
    (-0.3, -0.3, 3.14),
    (-3.0, -1.4, 0.39),
    (-0.4, 2.4, -1.53),
    (-2.6, 1.2, -0.57),
    (0.8, -2.0, 2.15),
    (-0.8, -1.8, 1.25),
    (-2.0, -0.2, -0.06),
    (0.4, 1.2, -2.0),
    (1.0, 0.0, 3.37),
    (-1.8, -1.2, 0.54),
]  # collision-free AND line-of-sight to P3 verified (incl. other packages)  # LOS-verified, collision-free


# ========================================================================
# Part II (v2) Configuration — Final Evaluation Extensions
# ========================================================================

# ---------------------------------------------------------------- Multi-robot ---
NUM_ROBOTS = 1                                 # Number of robots in scenario
ROBOT_ID = 1                                   # This robot's ID (set at runtime)

# Robot v2 dimensions (for manipulation capability)
# ROBOT_RADIUS, ROBOT_HEIGHT kept from v1; arm stows within footprint
ROBOT_V2_MASS = 8.0                           # kg (with arm)
ARM_STOWED_HEIGHT = 0.4                        # m (within footprint)

# ---------------------------------------------------------------- Sensor Suite v2 ---
# Sensor device names (extend v1)
REAR_CAMERA_NAME = "rear_camera"
IR_SENSOR_NAMES = {
    'front_left': "ir_fl",
    'front_right': "ir_fr",
    'left': "ir_l",
    'right': "ir_r",
    'rear_left': "ir_rl",
    'rear_right': "ir_rr",
    'front': "ir_f",
    'rear': "ir_r_center"
}
SONAR_SENSOR_NAMES = {
    'front': "sonar_f",
    'rear': "sonar_r",
    'left': "sonar_l",
    'right': "sonar_r"
}
LIGHT_SENSOR_NAME = "light_sensor"
BUMPER_NAMES = {
    'front': "bumper_f",
    'rear': "bumper_r"
}

# Sensor noise parameters (σ for Gaussian noise)
SENSOR_NOISE = {
    'gps_xy': 0.05,           # m
    'imu_yaw': 0.05,          # rad (~3 degrees)
    'lidar_range': 0.02,      # m
    'ir_range': 0.05,         # m
    'sonar_range': 0.03,      # m
    'camera_brightness': 0.0, # (quality metric, not noise)
}

# Sensor mounting transforms (robot frame, x forward y left)
SENSOR_POSITIONS = {
    'gps': (0.0, 0.0, 0.15),
    'imu': (0.0, 0.0, 0.05),
    'lidar': (0.0, 0.0, 0.20),
    'camera': (CAMERA_MOUNT_X, 0.0, CAMERA_MOUNT_Z),
    'rear_camera': (REAR_CAMERA_MOUNT_X, 0.0, CAMERA_MOUNT_Z),
    'ir_fl': (0.15, 0.10, 0.10),
    'ir_fr': (0.15, -0.10, 0.10),
    'ir_l': (0.0, 0.15, 0.10),
    'ir_r': (0.0, -0.15, 0.10),
    'ir_rl': (-0.15, 0.10, 0.10),
    'ir_rr': (-0.15, -0.10, 0.10),
    'sonar_f': (0.15, 0.0, 0.10),
    'sonar_r': (-0.15, 0.0, 0.10),
}

# ---------------------------------------------------------------- EKF Parameters ---
EKF_Q = [0.02**2, 0.02**2, (2*math.pi/180)**2]  # Process noise [x, y, theta]
EKF_R_GPS = [0.1**2, 0.1**2]                      # GPS measurement noise [x, y]
EKF_R_IMU = (3*math.pi/180)**2                    # IMU measurement noise [theta]
EKF_R_LANDMARK = [0.2**2, (10*math.pi/180)**2]    # Landmark [range, bearing]
EKF_GATE_GPS = 9.21                               # Chi-square 2 DOF @ 99%
EKF_GATE_IMU = 6.63                               # Chi-square 1 DOF @ 99%
EKF_GATE_LANDMARK = 9.21                          # Chi-square 2 DOF @ 99%
EKF_R_CAMERA_BEARING = (2 * math.pi / 180) ** 2   # camera marker bearing (lit), rad^2

# ---------------------------------------------------------------- Tracking & Fusion ---
TRACKER_CLUSTER_DISTANCE = 0.5                     # m
TRACKER_GATE_THRESHOLD = 4.0                       # Chi-square 2 DOF
TRACKER_MAX_AGE = 5                                # frames without update
TRACKER_MIN_HITS = 3                               # min hits to be valid
TRACKER_SPEED_THRESHOLD = 0.3                      # m/s for static/moving

OBSTACLE_AGREE_THRESHOLD = 0.3                     # m for VERIFIED
OBSTACLE_CONFLICT_THRESHOLD = 1.0                  # m for CONFLICT

# ---------------------------------------------------------------- DWA Parameters ---
DWA_V_SAMPLES = 10
DWA_W_SAMPLES = 20
DWA_DT = 0.2                                       # simulation timestep (s)
DWA_SIM_TIME = 2.0                                 # simulation horizon (s)
DWA_W_HEADING = 1.0
DWA_W_DISTANCE = 1.0
DWA_W_VELOCITY = 0.5
DWA_ALLOW_REVERSE = True

# ---------------------------------------------------------------- Rear Safety ---
REAR_CLEAR_DIST = 1.0                              # m, no reverse if closer
REAR_WARN_DIST = 2.0                               # m, warn threshold
REAR_CRITICAL_TTC = 3.0                            # s, yield threshold
REAR_WARN_TTC = 5.0                                # s, warning threshold

# ---------------------------------------------------------------- Costmap ---
COSTMAP_INFLATION_RADIUS = 0.25                    # m
COSTMAP_PREDICTION_HORIZON = 3.0                   # s for tracked obstacles

# ---------------------------------------------------------------- Recovery Behaviors ---
RECOVERY_WAIT_TIME = 2.0                           # s, wait before rotate
RECOVERY_ROTATE_SPEED = 0.3                        # rad/s
RECOVERY_BACKUP_DIST = 0.3                         # m
RECOVERY_TIMEOUT = 30.0                            # s, total recovery time

# ---------------------------------------------------------------- Manipulation (v2) ---
# Arm parameters (2R planar arm + lift)
ARM_LINK1_LENGTH = 0.25                            # m
ARM_LINK2_LENGTH = 0.20                            # m
ARM_LIFT_MIN = 0.0                                 # m
ARM_LIFT_MAX = 0.6                                 # m
ARM_JOINT_LIMITS = [                               # (min, max) rad
    (-math.pi, math.pi),      # shoulder
    (-math.pi/2, math.pi/2),  # elbow
]
ARM_MAX_VELOCITY = 1.0                             # rad/s
ARM_MAX_ACCELERATION = 2.0                         # rad/s^2

# Gripper parameters
GRIPPER_OPEN_WIDTH = 0.15                          # m
GRIPPER_CLOSE_FORCE = 10.0                         # N
GRIPPER_GRASP_WIDTH_MIN = 0.05                     # m
GRIPPER_GRASP_WIDTH_MAX = 0.12                     # m
GRIPPER_GRASP_FORCE_MIN = 2.0                      # N
# NOTE: the ARM_*/GRIPPER_*/DOCK_* constants above are the v2 (SCARA + lift)
# design placeholders of Phase 14; no module reads them. The manipulator that
# is actually built into the mission robot is configured in section 13 below.

# Docking tolerances
DOCK_LATERAL_TOL = 0.03                            # m
DOCK_HEADING_TOL = 3.0 * math.pi / 180             # rad

# ---------------------------------------------------------------- World v2 (Phase 13) ---
WORLD_V2_ARENA = (12.0, 8.0)                       # (width, depth) m
V2_ARENA_X_MIN, V2_ARENA_X_MAX = -6.0, 6.0
V2_ARENA_Y_MIN, V2_ARENA_Y_MAX = -4.0, 4.0
V2_INFLATION_RADIUS = 0.35                         # m
SHELF_LEVELS = 3
SHELF_BAYS = 3
SLOT_HEIGHT = 0.35                                 # m per level
PILE_LAYERS = 2
PILE_BOX_COUNT = 10                                # 10 boxes total in 2 layers

V2_SHELVES = {
    'SH1': {'pos': (-1.5, 2.4, 0.6), 'size': (2.4, 0.6, 1.2)},
    'SH2': {'pos': (2.5, 2.4, 0.6), 'size': (2.4, 0.6, 1.2)},
    'SH3': {'pos': (-1.5, -2.4, 0.6), 'size': (2.4, 0.6, 1.2)},
    'SH4': {'pos': (2.5, -2.4, 0.6), 'size': (2.4, 0.6, 1.2)},
}

V2_LANDMARKS = [
    (-5.0, 3.2), (0.0, 3.2), (5.0, 3.2),
    (-5.0, -3.2), (0.0, -3.2), (5.0, -3.2)
]

V2_RECEIVING_POS = (-4.5, 0.0)
V2_DISPATCH_ZONE_A = (4.8, 2.0)
V2_DISPATCH_ZONE_B = (4.8, -2.0)
V2_ROBOT_DOCKS = [
    (-4.8, 2.5, 0.0),
    (-4.8, -2.5, 0.0),
    (-4.8, -1.0, 0.0)
]

V2_STATIC_OBSTACLES = [
    (-1.5, 2.4, 2.4, 0.6),
    (2.5, 2.4, 2.4, 0.6),
    (-1.5, -2.4, 2.4, 0.6),
    (2.5, -2.4, 2.4, 0.6),
]

# ---------------------------------------------------------------- Communications ---
COMMS_RANGE = 20.0                                 # m, unlimited if None
COMMS_DROP_RATE = 0.0                              # 0-1, packet loss
COMMS_CHANNEL_HEARTBEAT = "heartbeat"
COMMS_CHANNEL_TASK = "task"
COMMS_CHANNEL_SLOT = "slot"
HEARTBEAT_INTERVAL = 1.0                           # s

# ---------------------------------------------------------------- Traffic ---
TRAFFIC_ONE_WAY_AISLES = []                        # List of aisle IDs
TRAFFIC_YIELD_DISTANCE = 1.0                       # m, yield at intersections
TRAFFIC_RESERVATION_HORIZON = 5.0                  # s, space-time planning

# ---------------------------------------------------------------- Fault Injection ---
# Scenarios defined in scenario_runner.py
# Fault types: 'lights_off', 'lights_dim', 'lidar_dropout', 'gps_dropout',
#              'camera_blind', 'stuck_at', 'bias', 'comm_loss'

# ---------------------------------------------------------------- Moving Obstacles ---
MOVING_OBSTACLE_LANES = {
    'patrol_north': [(-2.5, 2.4), (2.5, 2.4)],      # patrol lane endpoints (v1)
    'cross_aisle_1': [(0.8, -2.4), (0.8, 2.4)],    # crossing lane (v1)
    'patrol_north_v2': [(-4.5, 3.2), (4.5, 3.2)],  # patrol lane (v2)
    'cross_aisle_1_v2': [(1.5, -3.2), (1.5, 3.2)], # crossing lane (v2)
}
MOVING_OBSTACLE_SPEEDS = {
    'slow': 0.3,
    'normal': 0.5,
    'fast': 0.8
}
FAST_SPHERE_DEF = "FAST_SPHERE"
FAST_CART_DEF = "FAST_CART"
FAST_SPHERE_SPEED = 0.75
FAST_CART_SPEED = 0.70

CART_DEFS = {
    'CART_1': {'lane': 'patrol_north', 'speed': 0.4, 'axis': 0},
    'CART_2': {'lane': 'cross_aisle_1', 'speed': 0.3, 'axis': 1},
}

# ---------------------------------------------------------------- Random Seed ---
RANDOM_SEED = 42                                   # for reproducibility

# ---------------------------------------------------------------- Display Dashboard ---
DISPLAY_WIDTH = 320
DISPLAY_HEIGHT = 240
DISPLAY_UPDATE_RATE = 0.5                          # s
DASHBOARD_SHOW_COSTMAP = False                     # toggle for visualization

# ---------------------------------------------------------------- Mission KPIs ---
# Tracked in mission report
# - time, path_length, replans, collisions, min_clearance
# - grasp_success_rate, place_success_rate
# - sensor_health_events
# - ekf_rmse (vs ground truth)


# ========================================================================
# 13. Mobile manipulator on the mission robot (shoulder / elbow / wrist +
#     parallel gripper + latch).  Single source of truth for the world
#     generator (tools/generate_world.py), the kinematics (arm_kinematics.py),
#     the controllers (arm_controller.py, gripper.py, pick_place.py) and docking.
# ========================================================================
# Frames: robot frame FLU, origin on the floor under the wheel axle. The arm
# moves in the robot's x-z plane (y = 0). Joint angles are measured from the
# robot's +x axis, POSITIVE = UP (Webots hinge axis 0 -1 0). The gripper pitch
# phi = q1 + q2 + q3 is kept at 0 (gripper horizontal, pointing forward).
#
# Mount: a slim post at the rear of the body carries the shoulder. Placing it
# at the rear keeps the front camera, the front LiDAR sector and the front
# bumper completely free; the only LiDAR self-occlusion is a ~13 deg sector
# straight behind the robot (see LIDAR_SELF_FILTER_RANGE).
ARM_BASE_X = -0.115                           # m, post / shoulder x in robot frame
ARM_SHOULDER_Z = 0.25                         # m, shoulder axis height above floor
ARM_POST_SIZE = 0.025                         # m, square post cross-section
ARM_UPPER_LEN = 0.28                          # m, shoulder -> elbow (L1)
ARM_FORE_LEN = 0.25                           # m, elbow -> wrist   (L2)
ARM_LINK_THICKNESS = 0.03                     # m, link bar height (for clearance checks)
ARM_UPPER_PLATE_GAP = 0.07                    # m, clevis: upper arm = two plates this far apart
ARM_LINK_MASSES = (0.20, 0.15, 0.22)          # kg: upper arm, forearm, gripper body
# Counterweight (rear battery/ballast plate inside the body), modelled through
# the base's mass + centre of mass. Static tipping check with a 0.5 kg parcel
# held 0.40 m ahead: COM 0.04 m ahead of the axle vs. front caster at 0.11 m
# (without it: 0.09 m — too close). See docs/qa_prep.md.
ARM_COUNTERWEIGHT_MASS = 0.8                  # kg
ARM_COUNTERWEIGHT_X = -0.10                   # m, robot frame
ARM_MOTOR_NAMES = ("shoulder_motor", "elbow_motor", "wrist_motor")
ARM_SENSOR_NAMES = ("shoulder_sensor", "elbow_sensor", "wrist_sensor")
ARM_JOINT_LIMITS_RAD = (                      # (min, max); used range: tools/plot_arm_workspace.py
    (-0.20, 2.45),                            # shoulder (used 17..130 deg)
    (-3.05, 0.30),                            # elbow    (used -171..-70 deg; folds for carrying)
    (-0.60, 2.20),                            # wrist    (used -11..100 deg)
)
ARM_JOINT_MAX_VEL = (1.0, 1.2, 1.5)           # rad/s (Webots maxVelocity, also trajectory limit)
ARM_JOINT_MAX_TORQUE = (25.0, 15.0, 6.0)      # N m  (Webots maxTorque)
ARM_JOINT_MAX_ACC = 2.0                       # rad/s^2, quintic trajectory limit
ARM_JOINT_TOL = 0.02                          # rad, "joint at target" tolerance
ARM_JOINT_TOL_LOADED = 0.035                  # rad, accepted residual once the arm has STOPPED
ARM_SETTLE_STILL_TIME = 0.3                   # s of no joint motion that counts as "stopped"
ARM_SETTLE_STILL_TOL = 1e-3                   # rad, max joint drift inside that window
ARM_TRACKING_ERROR_MAX = 0.25                 # rad, |q_cmd - q_meas| that counts as blocked
ARM_TRACKING_FAULT_TIME = 0.4                 # s the error must persist -> fault
ARM_MOTION_TIMEOUT_MARGIN = 3.0               # s added to every planned motion duration
ARM_STEP_TIMEOUT_SLACK = 2.0                  # s, step deadline beyond the arm's own settle timeout
ARM_STEP_TIMEOUT_DEFAULT = 15.0               # s, any step without an arm motion or own timeout
ARM_SENSOR_WAIT_TIMEOUT = 2.0                 # s, max wait for valid joint encoders at start-up
GRIPPER_STEP_TIMEOUT = 8.0                    # s, open/close step (gripper faults at travel time + 3 s)
ARM_CART_STEP = 0.01                          # m, Cartesian line interpolation step
ARM_CART_SPEED = 0.10                         # m/s, nominal TCP speed on straight lines

# Gripper (two LinearMotor fingers closing along robot y) ----------------
GRIPPER_WRIST_TO_PALM = 0.05                  # m, wrist axis -> palm face
GRIPPER_PALM_GAP = 0.01                       # m, palm face -> parcel face when grasped
# Tool centre point (TCP) = centre of a grasped parcel
ARM_TCP_OFFSET = GRIPPER_WRIST_TO_PALM + GRIPPER_PALM_GAP + PKG_SIZE / 2.0   # 0.16 m
GRIPPER_PALM_SIZE = (0.02, 0.30, 0.05)        # m (x, y, z) palm plate
GRIPPER_OPEN_HALF_WIDTH = 0.13                # m, finger inner face from centre when open
GRIPPER_FINGER_TRAVEL = 0.12                  # m, per finger (fully closed width 0.02 m)
GRIPPER_FINGER_SIZE = (0.12, 0.015, 0.04)     # m (length, thickness, height)
GRIPPER_FINGER_MASS = 0.03                    # kg
GRIPPER_FINGER_MAX_VEL = 0.05                 # m/s
GRIPPER_FINGER_MAX_FORCE = 30.0               # N (Webots maxForce)
GRIPPER_GRIP_FORCE = 12.0                     # N, available force while closing (force-limited)
GRIPPER_POS_TOL = 0.003                       # m, finger "at target"
GRIPPER_STALL_TIME = 0.4                      # s without finger motion = stalled on an object
GRIPPER_STALL_WINDOW_TOL = 1e-3               # m, max finger travel inside the stall window
GRIPPER_CONTACT_DONE_TIME = 0.15              # s of pad contact + stable width = closed on a parcel
GRIPPER_WIDTH_TOL = 0.015                     # m, |width - PKG_SIZE| for "holding"
GRIPPER_FORCE_MIN = 2.0                       # N, per-finger contact force for "holding"
GRIPPER_PALM_RANGE_MAX = 0.50                 # m, palm range-finder max range
GRIPPER_PALM_PRESENT_MAX = 0.03               # m, palm range <= this -> parcel at the palm
GRIPPER_FINGER_MOTOR_NAMES = ("finger_left_motor", "finger_right_motor")
GRIPPER_FINGER_SENSOR_NAMES = ("finger_left_sensor", "finger_right_sensor")
GRIPPER_FINGER_TOUCH_NAMES = ("finger_left_touch", "finger_right_touch")
GRIPPER_PALM_RANGE_NAME = "palm_range"
GRIPPER_CONNECTOR_NAME = "gripper_latch"
# Latch = Webots Connector pair (active on the palm, passive on each parcel face).
CONNECTOR_MODEL = "parcel_latch"
CONNECTOR_DISTANCE_TOL = 0.04                 # m between connector origins
CONNECTOR_AXIS_TOL = 0.35                     # rad between the (opposed) x-axes
CONNECTOR_LOCK_TIMEOUT = 1.0                  # s to see isLocked after lock()

# Named arm poses: TCP (x, z) in the robot frame, gripper horizontal --------
# Stow/carry: parcel held over the body, elbow folded straight up — parcel
# bottom at z = 0.18 m (LiDAR plane 0.14 m and both lenses 0.12 m stay clear),
# parcel front 0.015 m past the bumper line, elbow above the body.
ARM_STOW_TCP = (0.085, 0.28)
ARM_SAFE_TRANSIT_Z = 0.27                     # m, TCP height before swinging a parcel over the body
ARM_PREGRASP_BACKOFF = 0.03                   # m, TCP behind the grasp point (x); wrist stays 1.5 cm off the bumper
ARM_PREGRASP_RAISE = 0.14                     # m, TCP above the grasp point (z) — fingers clear the top
ARM_LIFT_CHECK_DZ = 0.05                      # m, small lift used to verify the attachment
ARM_PLACE_HOVER = 0.015                       # m, parcel bottom above the shelf while inserting
ARM_PLACE_RELEASE_DROP = 0.003                # m, parcel bottom above the shelf at release
ARM_PLACE_CLEARANCE = 0.02                    # m, parcel front <-> rack front before inserting
ARM_RETRACT_DIST = 0.06                       # m, palm retreat used to verify the release
ARM_VERIFY_RANGE_TOL = 0.02                   # m, palm-range check tolerance after retreat
ARM_SETTLE_TIME = 0.5                         # s, wait after release before verifying
ARM_MAX_GRASP_ATTEMPTS = 2                    # dock + grasp attempts before ERROR

# Docking (fine positioning of the base before manipulation) --------------
ARM_DOCK_PICK_STANDOFF = 0.30                 # m, LiDAR centre -> parcel near face
ARM_DOCK_PLACE_STANDOFF = 0.25                # m, LiDAR centre -> rack front (kick plate)
ARM_DOCK_PREDOCK_EXTRA = 0.20                 # m, pre-dock point further back on the line
ARM_DOCK_RANGE_TOL = 0.008                    # m
ARM_DOCK_LATERAL_TOL = 0.015                  # m (open gripper clears the parcel by 0.03 m/side)
ARM_DOCK_HEADING_TOL = math.radians(1.5)      # rad
ARM_DOCK_POINT_TOL = 0.012                    # m, pre-dock point stabilisation
ARM_DOCK_V_MAX = 0.06                         # m/s
ARM_DOCK_W_MAX = 0.6                          # rad/s
ARM_DOCK_K_RANGE = 0.8                        # 1/s, creep gain on range error
ARM_DOCK_K_HEADING = 2.0                      # 1/s, heading gain
ARM_DOCK_K_LATERAL = 3.0                      # rad/m, heading offset per metre of lateral error (creep)
ARM_DOCK_RANGE_WINDOW = math.radians(3.0)     # rad, +- LiDAR window used for the face range
ARM_DOCK_MIN_RANGE = 0.20                     # m, never creep closer than this (abort)
ARM_DOCK_SETTLE_TIME = 0.6                    # s, base stationary before the arm moves
ARM_DOCK_SETTLE_MOTION = 0.002                # m, max pose change during settle
ARM_DOCK_TIMEOUT = 45.0                       # s
ARM_DOCK_MAX_REFINE_SHIFT = 0.30              # m, LiDAR face fit may move the estimate this much
ARM_DOCK_FACE_JUMP = 0.03                     # m, range discontinuity that ends the face segment
ARM_DOCK_REFINE_WINDOW = math.radians(20.0)   # rad, +- search window around the estimate bearing
