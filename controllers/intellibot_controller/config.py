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
# Empirically measured on the installed Webots (see docs/webots_findings.md).
# Assumption to verify: ray 0 points forward (+x), index increases CCW.
LIDAR_ANGLE_SIGN = 1.0                        # +1 if CCW index order
LIDAR_ANGLE_OFFSET = 0.0                      # rad added to computed angle

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
MIN_CONTOUR_AREA = 150.0                      # px^2 at 320x240
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
