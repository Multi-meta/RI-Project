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

# Camera
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_FOV = 1.0                              # horizontal FOV, rad
CAMERA_MOUNT_X = 0.15
CAMERA_MOUNT_Z = 0.12

# Detector (OpenCV hue 0-180 scale). Kept out of the yellow(~30)/cyan(~90)
# hue bands so zone patches can never be confused with packages.
HSV_RANGES = {
    "red":   [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    "green": [((40, 80, 60), (80, 255, 255))],
    "blue":  [((105, 80, 60), (135, 255, 255))],
}
MIN_CONTOUR_AREA = 100.0                      # px^2 at 320x240 (P3 at ~4 m is ~14 px wide)
ASPECT_MIN, ASPECT_MAX = 0.6, 1.6             # bbox aspect ratio window

# Package position estimate
PKG_BEARING_WINDOW = math.radians(3.0)        # lidar min-range window around bearing

# ---------------------------------------------------------------- 8. FSM ---
FIND_ROT_SPEED = 0.6                          # rad/s scan rotation
FIND_MAX_TURNS = 2.0                          # full revolutions before timeout
DETECTION_CONSEC = 5                          # consecutive frames to accept
APPROACH_DISTANCE = 0.45                      # m, stand-off from package
PICK_PAUSE_S = 1.0                            # s, stop duration in PICK

# ---------------------------------------------------------------- 9. Task ---
TASK_PACKAGE = "P3"
TASK_PACKAGE_DEF = "PKG_P3"
TASK_PACKAGE_COLOR = "blue"
TASK_ZONE = "B"
AUTO_QUIT = True                              # simulationQuit(0) at DONE

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
    'camera': (0.15, 0.0, 0.15),
    'rear_camera': (-0.15, 0.0, 0.15),
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

# Docking tolerances
DOCK_LATERAL_TOL = 0.03                            # m
DOCK_HEADING_TOL = 3.0 * math.pi / 180             # rad

# ---------------------------------------------------------------- World v2 (Phase 13) ---
# Not used yet - placeholders for Phase 13
WORLD_V2_ARENA = (12.0, 8.0)                       # (width, depth) m
SHELF_LEVELS = 3
SHELF_BAYS = 3
SLOT_HEIGHT = 0.25                                 # m per level
PILE_LAYERS = 2
PILE_BOX_COUNT = (9, 12)                           # (min, max) boxes

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
    'patrol_north': [(0.0, 2.5), (7.0, 2.5)],      # patrol lane endpoints
    'cross_aisle_1': [(2.0, -3.0), (2.0, 3.0)],    # crossing lane
}
MOVING_OBSTACLE_SPEEDS = {
    'slow': 0.3,
    'normal': 0.5,
    'fast': 0.8
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
