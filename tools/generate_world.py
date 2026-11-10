"""generate_world.py — write the warehouse worlds from config.py.

The world is GENERATED so the simulated world and the occupancy grid can
never disagree (project_summary.md rule 3). Everything is built from
primitive nodes (Solid / Shape / Box / Cylinder / Sphere) to avoid
EXTERNPROTO dependencies across Webots versions.

Two worlds are written, identical except for the robot's controller:
    worlds/warehouse.wbt             controller "intellibot_controller" (mission)
    worlds/warehouse_drive_test.wbt  controller "drive_test" (Phase 3 validation;
                                     mode via DRIVE_TEST_MODE env var)

Run:  python tools/generate_world.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "controllers", "intellibot_controller"))
import config  # noqa: E402
import shelving  # noqa: E402
from arm_kinematics import PlanarPitchArm  # noqa: E402

WORLDS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                          "worlds")
WORLDS = {
    "warehouse.wbt": "intellibot_controller",
    "warehouse_drive_test.wbt": "drive_test",
    "warehouse_manip_check.wbt": "manip_check",      # arm / gripper / camera checks
    "warehouse_v2.wbt": "intellibot_controller",
}

# Webots RGB 0..1
VIEW_ANGLED = config.VIEW_ANGLED

COLORS = {
    "floor": (0.5, 0.5, 0.5),
    "wall": (0.75, 0.75, 0.75),
    "shelf": (0.25, 0.25, 0.25),
    "crate": (0.45, 0.28, 0.1),
    "pickup": (0.9, 0.9, 0.9),
    "zone_a": (0.95, 0.9, 0.1),     # yellow
    "zone_b": (0.1, 0.85, 0.9),     # cyan
    "red": (1.0, 0.0, 0.0),
    "green": (0.0, 1.0, 0.0),
    "blue": (0.0, 0.0, 1.0),
    "body": (0.15, 0.35, 0.7),
    "robot_1": (0.10, 0.20, 0.50),     # navy blue
    "robot_2": (0.10, 0.45, 0.20),     # forest green
    "wheel": (0.1, 0.1, 0.1),
    "caster": (0.3, 0.3, 0.3),
    "sensor": (0.05, 0.05, 0.05),
    "bumper": (0.15, 0.15, 0.15),
    "barrel": (0.75, 0.4, 0.05),
    "cart_1": (0.95, 0.85, 0.1),
    "cart_2": (0.90, 0.45, 0.1),
    "ball": (0.45, 0.45, 0.50),  # DESATURATED gray-blue: must stay outside every HSV package window (a saturated blue ball was mistaken for package P3!)
    "steel": (0.70, 0.72, 0.75),       # metallic arm silver/steel
    "gripper_red": (0.85, 0.10, 0.10), # signal red
}

CASTER_MATERIAL = "caster"          # zero-friction contact material
PAD_MATERIAL = "gripper_pad"        # finger pads (high friction vs parcels)
PARCEL_MATERIAL = "parcel"


def f(v):
    """Compact float formatting for VRML."""
    return f"{float(v):.6g}"


def vec(*vals):
    return " ".join(f(v) for v in vals)


def camera_node(name, mount_x, yaw, fov):
    """Planar Webots camera on the robot (FLU): looks along its own +x.

    The housing box is placed BEHIND the lens (-x in the camera frame) for
    both cameras. (The previous rear camera had its housing at +0.01 in its
    own, rotated frame, i.e. in FRONT of the lens.)
    """
    hd, hw, hh = config.CAMERA_HOUSING_SIZE
    housing = (f"Pose {{ translation {f(-hd / 2.0)} 0 0 children [ Shape {{ "
               f"appearance {mat(COLORS['sensor'])} geometry Box {{ size "
               f"{vec(hd, hw, hh)} }} }} ] }}")
    rot = f"      rotation 0 0 1 {f(yaw)}\n" if abs(yaw) > 1e-9 else ""
    return (f"    Camera {{\n"
            f"      translation {vec(mount_x, 0, config.CAMERA_MOUNT_Z)}\n"
            f"{rot}"
            f"      children [ {housing} ]\n"
            f"      name \"{name}\"\n"
            f"      fieldOfView {f(fov)}\n"
            f"      width {config.CAMERA_WIDTH}\n"
            f"      height {config.CAMERA_HEIGHT}\n"
            f"      projection \"{config.CAMERA_PROJECTION}\"\n"
            f"      near {f(config.CAMERA_NEAR)}\n"
            f"      recognition Recognition {{ frameColor 0.93 0.83 0 }}\n"
            f"    }}")


def _box(size, color, translation=(0, 0, 0), rotation=None):
    rot = f" rotation {vec(*rotation)}" if rotation else ""
    return (f"Pose {{ translation {vec(*translation)}{rot} children [ Shape {{ "
            f"appearance {mat(color)} geometry Box {{ size {vec(*size)} }} }} ] }}")


def _bbox(size, translation=(0, 0, 0)):
    return f"Pose {{ translation {vec(*translation)} children [ Box {{ size {vec(*size)} }} ] }}"


def _hub(radius, width, color, translation=(0, 0, 0)):
    """Joint hub: cylinder whose axis is the robot y axis."""
    return (f"Pose {{ translation {vec(*translation)} rotation 1 0 0 1.5708 children [ "
            f"Shape {{ appearance {mat(color)} geometry Cylinder {{ height {f(width)} "
            f"radius {f(radius)} subdivision 16 }} }} ] }}")


def pitch_arm_vrml(indent="    "):
    """Shoulder/elbow/wrist pitch arm + parallel gripper + latch (config section 13).

    Kinematic structure (robot frame FLU, every hinge axis 0 -1 0 so a
    positive angle LIFTS the link — same convention as arm_kinematics.py):

        post (fixed) -> shoulder HingeJoint -> upper arm (clevis, 2 plates)
          -> elbow HingeJoint -> forearm -> wrist HingeJoint -> gripper body
             (palm plate, Connector "gripper_latch", DistanceSensor
              "palm_range") -> 2 x SliderJoint fingers with force pads

    Each link Solid's frame sits on its joint axis (endPoint translation ==
    anchor), so the initial pose is simply rotation (0 -1 0 q). The arm is
    generated in the STOW pose (q from inverse kinematics of ARM_STOW_TCP),
    with matching joint positions, so the simulation starts stowed.
    """
    arm = PlanarPitchArm.from_config()
    q = arm.inverse(*config.ARM_STOW_TCP, 0.0)
    if q is None:
        raise SystemExit("ARM_STOW_TCP is not reachable — check config section 13")
    L1, L2 = config.ARM_UPPER_LEN, config.ARM_FORE_LEN
    th = config.ARM_LINK_THICKNESS
    gap = config.ARM_UPPER_PLATE_GAP
    m_up, m_fore, m_grip = config.ARM_LINK_MASSES
    steel, dark = COLORS["steel"], COLORS["sensor"]
    lims = config.ARM_JOINT_LIMITS_RAD

    def motor(i):
        lo, hi = lims[i]
        return (f"RotationalMotor {{ name \"{config.ARM_MOTOR_NAMES[i]}\" "
                f"maxVelocity {f(config.ARM_JOINT_MAX_VEL[i])} "
                f"maxTorque {f(config.ARM_JOINT_MAX_TORQUE[i])} "
                f"minPosition {f(lo)} maxPosition {f(hi)} sound \"\" }}\n"
                f"        PositionSensor {{ name \"{config.ARM_SENSOR_NAMES[i]}\" }}")

    def hinge_params(i, anchor):
        lo, hi = lims[i]
        return (f"HingeJointParameters {{ axis 0 -1 0 anchor {vec(*anchor)} "
                f"position {f(q[i])} minStop {f(lo - 0.05)} maxStop {f(hi + 0.05)} }}")

    # ---------------------------------------------------------- gripper
    w2p = config.GRIPPER_WRIST_TO_PALM
    pt, pw, ph = config.GRIPPER_PALM_SIZE
    fl, ft, fh = config.GRIPPER_FINGER_SIZE
    pad_t = 0.004
    finger_y = config.GRIPPER_OPEN_HALF_WIDTH + ft / 2.0 + pad_t
    fx = w2p + fl / 2.0
    travel = config.GRIPPER_FINGER_TRAVEL

    def finger(side):
        sgn = 1.0 if side == "left" else -1.0
        idx = 0 if side == "left" else 1
        pad_rot = (0, 0, 1, -1.5708 * sgn)      # pad x-axis = closing direction
        pad_y = -sgn * (ft / 2.0 + pad_t / 2.0)
        return f"""SliderJoint {{
                jointParameters JointParameters {{ axis 0 {f(-sgn)} 0 minStop -0.002 maxStop {f(travel + 0.002)} }}
                device [
                  LinearMotor {{ name "{config.GRIPPER_FINGER_MOTOR_NAMES[idx]}" maxVelocity {f(config.GRIPPER_FINGER_MAX_VEL)} maxForce {f(config.GRIPPER_FINGER_MAX_FORCE)} minPosition 0 maxPosition {f(travel)} sound "" }}
                  PositionSensor {{ name "{config.GRIPPER_FINGER_SENSOR_NAMES[idx]}" }}
                ]
                endPoint Solid {{
                  translation {vec(fx, sgn * finger_y, 0)}
                  children [
                    Shape {{ appearance {mat(COLORS['gripper_red'])} geometry Box {{ size {vec(fl, ft, fh)} }} }}
                    TouchSensor {{
                      translation {vec(0, pad_y, 0)}
                      rotation {vec(*pad_rot)}
                      children [ Shape {{ appearance {mat(dark)} geometry Box {{ size {vec(pad_t, fl, fh)} }} }} ]
                      name "{config.GRIPPER_FINGER_TOUCH_NAMES[idx]}"
                      type "force"
                      lookupTable [ 0 0 0, 1000 1000 0 ]
                      contactMaterial "{PAD_MATERIAL}"
                      boundingObject Box {{ size {vec(pad_t, fl, fh)} }}
                      physics Physics {{ density -1 mass 0.005 }}
                    }}
                  ]
                  name "finger_{side}"
                  boundingObject Box {{ size {vec(fl, ft, fh)} }}
                  physics Physics {{ density -1 mass {f(config.GRIPPER_FINGER_MASS)} }}
                }}
              }}"""

    bracket = w2p - pt
    gripper = f"""Solid {{
              translation {vec(L2, 0, 0)}
              rotation 0 -1 0 {f(q[2])}
              children [
                {_hub(0.02, 0.05, dark)}
                {_box((bracket, 0.04, 0.04), steel, (bracket / 2.0, 0, 0))}
                {_box((pt, pw, ph), dark, (w2p - pt / 2.0, 0, 0))}
                Connector {{
                  translation {vec(w2p, 0, 0)}
                  name "{config.GRIPPER_CONNECTOR_NAME}"
                  model "{config.CONNECTOR_MODEL}"
                  type "active"
                  autoLock FALSE
                  unilateralLock TRUE
                  unilateralUnlock TRUE
                  distanceTolerance {f(config.CONNECTOR_DISTANCE_TOL)}
                  axisTolerance {f(config.CONNECTOR_AXIS_TOL)}
                  rotationTolerance {f(config.CONNECTOR_AXIS_TOL)}
                  numberOfRotations 0
                  snap FALSE
                }}
                DistanceSensor {{
                  translation {vec(w2p + 0.0005, 0, 0)}
                  name "{config.GRIPPER_PALM_RANGE_NAME}"
                  type "generic"
                  numberOfRays 1
                  lookupTable [ 0 0 0, {f(config.GRIPPER_PALM_RANGE_MAX)} {f(config.GRIPPER_PALM_RANGE_MAX)} 0 ]
                }}
                {finger("left")}
                {finger("right")}
              ]
              name "gripper"
              boundingObject Group {{ children [ {_bbox((bracket, 0.04, 0.04), (bracket / 2.0, 0, 0))} {_bbox((pt, pw, ph), (w2p - pt / 2.0, 0, 0))} ] }}
              physics Physics {{ density -1 mass {f(m_grip)} centerOfMass [ {vec(w2p * 0.7, 0, 0)} ] }}
            }}"""

    forearm = f"""Solid {{
          translation {vec(L1, 0, 0)}
          rotation 0 -1 0 {f(q[1])}
          children [
            {_hub(0.022, 0.05, dark)}
            {_box((L2, 0.03, th), steel, (L2 / 2.0, 0, 0))}
            HingeJoint {{
              jointParameters {hinge_params(2, (L2, 0, 0))}
              device [
                {motor(2)}
              ]
              endPoint {gripper}
            }}
          ]
          name "forearm"
          boundingObject {_bbox((L2, 0.03, th), (L2 / 2.0, 0, 0))}
          physics Physics {{ density -1 mass {f(m_fore)} centerOfMass [ {vec(L2 / 2.0, 0, 0)} ] }}
        }}"""

    plate_t = 0.012
    upper = f"""Solid {{
      translation {vec(config.ARM_BASE_X, 0, config.ARM_SHOULDER_Z)}
      rotation 0 -1 0 {f(q[0])}
      children [
        {_hub(0.022, gap + plate_t, dark)}
        {_box((L1 + 0.04, plate_t, th), steel, (L1 / 2.0, gap / 2.0, 0))}
        {_box((L1 + 0.04, plate_t, th), steel, (L1 / 2.0, -gap / 2.0, 0))}
        HingeJoint {{
          jointParameters {hinge_params(1, (L1, 0, 0))}
          device [
            {motor(1)}
          ]
          endPoint {forearm}
        }}
      ]
      name "upper_arm"
      boundingObject Group {{ children [ {_bbox((L1, plate_t, th), (L1 / 2.0, gap / 2.0, 0))} {_bbox((L1, plate_t, th), (L1 / 2.0, -gap / 2.0, 0))} ] }}
      physics Physics {{ density -1 mass {f(m_up)} centerOfMass [ {vec(L1 / 2.0, 0, 0)} ] }}
    }}"""

    return f"""{indent}HingeJoint {{
      jointParameters {hinge_params(0, (config.ARM_BASE_X, 0, config.ARM_SHOULDER_Z))}
      device [
        {motor(0)}
      ]
      endPoint {upper}
    }}"""


def parcel_connectors():
    """Passive latch on each vertical face of a parcel (x-axis pointing out)."""
    h = config.PKG_SIZE / 2.0
    faces = (("px", (h, 0, 0), 0.0), ("nx", (-h, 0, 0), 3.14159),
             ("py", (0, h, 0), 1.5708), ("ny", (0, -h, 0), -1.5708))
    out = []
    for tag, t, yaw in faces:
        out.append(f"""    Connector {{
      translation {vec(*t)}
      rotation 0 0 1 {f(yaw)}
      name "latch_{tag}"
      model "{config.CONNECTOR_MODEL}"
      type "passive"
      distanceTolerance {f(config.CONNECTOR_DISTANCE_TOL)}
      axisTolerance {f(config.CONNECTOR_AXIS_TOL)}
      rotationTolerance {f(config.CONNECTOR_AXIS_TOL)}
      numberOfRotations 0
      snap FALSE
    }}""")
    return "\n".join(out)


def build_rack(rack):
    """Warehouse-style destination rack (shelving.Rack), static, with collision.

    Local frame: origin at the rack's floor centre; the open front faces -x.
    Structure: 6 square uprights (front/back x 3 bay edges), one platform per
    level, a front kick plate up to level 1 (the LiDAR plane hits it, which
    gives docking an exact range reference) and a back panel. Thin dark strips
    on the front edge mark every slot (visual only, no collision).
    """
    D = config.DEST_RACK_DEPTH
    W = config.DEST_RACK_WIDTH
    post = config.DEST_RACK_POST_SIZE
    th = config.DEST_RACK_PLATFORM_THICKNESS
    pt = config.DEST_RACK_PANEL_THICKNESS
    tops = config.DEST_RACK_LEVEL_TOPS
    H = tops[-1]
    frame, shelf = config.DEST_RACK_FRAME_COLOR, config.DEST_RACK_SHELF_COLOR
    ys = [-W / 2.0 + post / 2.0 + i * config.DEST_RACK_BAY_WIDTH
          for i in range(config.DEST_RACK_BAYS + 1)]
    shapes, bounds = [], []
    for x in (-D / 2.0 + post / 2.0, D / 2.0 - post / 2.0):
        for y in ys:
            shapes.append(_box((post, post, H), frame, (x, y, H / 2.0)))
            bounds.append(_bbox((post, post, H), (x, y, H / 2.0)))
    for top in tops:
        shapes.append(_box((D, W, th), shelf, (0, 0, top - th / 2.0)))
        bounds.append(_bbox((D, W, th), (0, 0, top - th / 2.0)))
    kick_h = tops[0] - th
    shapes.append(_box((pt, W, kick_h), frame, (-D / 2.0 + pt / 2.0, 0, kick_h / 2.0)))
    bounds.append(_bbox((pt, W, kick_h), (-D / 2.0 + pt / 2.0, 0, kick_h / 2.0)))
    shapes.append(_box((pt, W, H), frame, (D / 2.0 - pt / 2.0, 0, H / 2.0)))
    bounds.append(_bbox((pt, W, H), (D / 2.0 - pt / 2.0, 0, H / 2.0)))
    cy = rack.center[1]
    for slot in rack.slots:
        if not slot.reachable:
            continue
        shapes.append(_box((0.004, 0.10, 0.012), COLORS["sensor"],
                           (-D / 2.0 - 0.002, slot.center[1] - cy, slot.surface_z - th / 2.0)))
    children = "\n    ".join(shapes)
    bchildren = " ".join(bounds)
    return (f"DEF {rack.def_name} Solid {{\n"
            f"  translation {vec(rack.center[0], rack.center[1], 0)}\n"
            f"  children [\n    {children}\n  ]\n"
            f"  name \"{rack.def_name.lower()}\"\n"
            f"  boundingObject Group {{ children [ {bchildren} ] }}\n"
            f"}}\n")


def mat(color):
    return (f"Appearance {{ material Material {{ diffuseColor "
            f"{color[0]:g} {color[1]:g} {color[2]:g} }} }}")


def solid(defname, translation, size, color, physics=None, bounding=True):
    """Axis-aligned box Solid. `name` = lower-case DEF so names are unique."""
    s = f"DEF {defname} Solid {{\n"
    s += f"  translation {translation}\n"
    s += ("  children [\n"
          f"    Shape {{ appearance {mat(color)} "
          f"geometry Box {{ size {size} }} }}\n"
          "  ]\n")
    s += f"  name \"{defname.lower()}\"\n"
    if bounding:
        s += f"  boundingObject Box {{ size {size} }}\n"
    if physics:
        s += f"  physics {physics}\n"
    s += "}\n"
    return s


def build_decoy():
    """Yellow look-alike robot (DEF config.DECOY_DEF): IntelliBot-sized body,
    two wheels, casters, LiDAR puck, camera bar and a beacon — all VISUAL. It
    is a plain kinematic Solid (no Robot node, no devices, no Physics): the
    supervisor moves it along config.DECOY_LANE and spins the wheels / flashes
    the beacon (moving_obstacles.DecoyBot). It never picks or places anything.
    Body frame: x forward, origin on the floor between the wheels."""
    d = config.DECOY_DEF
    L, W, H = config.DECOY_BODY
    yellow, black, dark, grey = config.DECOY_COLOR, (0.05, 0.05, 0.05), (0.18, 0.18, 0.18), (0.55, 0.55, 0.58)
    z0 = 0.03                                         # body underside clearance
    zc = z0 + H / 2
    (x0, y0), (x1, y1) = config.DECOY_LANE
    yaw = math.atan2(y1 - y0, x1 - x0)                # starts at end 0, facing end 1

    def shape(geom, color):
        return f"Shape {{ appearance {mat(color)} geometry {geom} }}"

    def pose(t, child, rot=None):
        r = f" rotation {rot}" if rot else ""
        return f"Pose {{ translation {t}{r} children [ {child} ] }}"

    wr, ww, wy = 0.05, 0.03, W / 2 + 0.015           # wheel radius / width / axle offset

    def wheel(side, y):
        tire = pose("0 0 0", shape(f"Cylinder {{ radius {wr} height {ww} subdivision 24 }}", black),
                    rot="1 0 0 1.5708")
        spokes = (shape(f"Box {{ size {1.4 * wr:.3f} {ww + 0.004:.3f} 0.012 }}", grey) + " "
                  + shape(f"Box {{ size 0.012 {ww + 0.004:.3f} {1.4 * wr:.3f} }}", grey))
        return (f"DEF {d}_WHEEL_{side} Pose {{ translation 0 {y:.3f} {wr} rotation 0 1 0 0 "
                f"children [ {tire} {spokes} ] }}")

    beacon = (f"Shape {{ appearance Appearance {{ material DEF {d}_BEACON Material {{ "
              f"diffuseColor 1 0.55 0 emissiveColor 0.45 0.25 0 }} }} "
              f"geometry Cylinder {{ radius 0.018 height 0.03 subdivision 16 }} }}")
    parts = [
        pose(f"0 0 {zc:.3f}", shape(f"Box {{ size {L} {W} {H} }}", yellow)),                     # body
        pose(f"{L / 2 + 0.004:.3f} 0 {z0 + 0.03:.3f}", shape(f"Box {{ size 0.012 {W + 0.01:.3f} 0.035 }}", black)),   # front bumper
        pose(f"{-L / 2 - 0.004:.3f} 0 {z0 + 0.03:.3f}", shape(f"Box {{ size 0.012 {W + 0.01:.3f} 0.035 }}", black)),  # rear bumper
        pose(f"0 0 {z0 + H + 0.003:.3f}", shape(f"Box {{ size {L - 0.06:.3f} {W - 0.05:.3f} 0.006 }}", dark)),        # top deck
        pose(f"0.03 0 {z0 + H + 0.024:.3f}", shape("Cylinder { radius 0.04 height 0.036 subdivision 20 }", black)),   # LiDAR puck
        pose(f"0.03 0 {z0 + H + 0.008:.3f}", shape("Cylinder { radius 0.043 height 0.006 subdivision 20 }", grey)),
        pose(f"{L / 2 + 0.002:.3f} 0 {z0 + H - 0.04:.3f}", shape("Box { size 0.008 0.09 0.026 }", black)),           # camera bar
        pose(f"{-L / 2 + 0.05:.3f} 0 {z0 + H + 0.025:.3f}", shape("Cylinder { radius 0.007 height 0.05 subdivision 8 }", grey)),  # mast
        pose(f"{-L / 2 + 0.05:.3f} 0 {z0 + H + 0.065:.3f}", beacon),
        pose(f"{L / 2 - 0.04:.3f} 0 0.02", shape("Sphere { radius 0.02 subdivision 2 }", grey)),   # casters
        pose(f"{-L / 2 + 0.04:.3f} 0 0.02", shape("Sphere { radius 0.02 subdivision 2 }", grey)),
        wheel("L", wy), wheel("R", -wy),
    ]
    bound = (f"Group {{ children [ Pose {{ translation 0 0 {zc:.3f} children [ Box {{ size {L} {W} {H} }} ] }} "
             f"Pose {{ translation 0 0 {wr} rotation 1 0 0 1.5708 children [ Cylinder {{ radius {wr} "
             f"height {W + 2 * ww + 0.01:.3f} }} ] }} ] }}")
    body = "\n    ".join(parts)
    return (f"DEF {d} Solid {{\n"
            f"  translation {x0} {y0} 0\n"
            f"  rotation 0 0 1 {yaw:.4f}\n"
            f"  children [\n    {body}\n  ]\n"
            f"  name \"{d.lower()}\"\n"
            f"  boundingObject {bound}\n"
            f"}}\n")


def build_robot(controller, def_name="INTELLIBOT", robot_name="INTELLIBOT",
                start_pos=None, start_rot=None, controller_args=None,
                body_color=None, has_arm=False, with_arm=False):
    """Differential-drive robot.

    with_arm : mission robot's shoulder/elbow/wrist arm + gripper (config 13)
    has_arm  : the v2 lift + SCARA design study (Phase 14, warehouse_v2.wbt)
    """
    if start_pos is None:
        start_pos = (config.ROBOT_START[0], config.ROBOT_START[1], 0)
    if start_rot is None:
        start_rot = (0, 0, 1, config.ROBOT_START[2])
    if body_color is None:
        body_color = COLORS["body"]

    if has_arm:
        bl, bw, bh = 0.55, 0.40, 0.18
        body_mass = 18.0
        r, L = 0.04, 0.33
        cr = 0.03
    else:
        bl, bw, bh = config.BODY_LENGTH, config.BODY_WIDTH, config.BODY_HEIGHT
        body_mass = config.BODY_MASS
        r, L = config.WHEEL_RADIUS, config.WHEEL_SEPARATION
        cr = config.CASTER_RADIUS

    body_z = cr + bh / 2.0
    caster_x = bl / 2.0 - 0.04

    def cyl_y(radius, height, color):
        """Cylinder with its axis along robot y (wheel orientation)."""
        return (f"Pose {{ rotation 1 0 0 1.5708 children [ "
                f"Shape {{ appearance {mat(color)} geometry "
                f"Cylinder {{ height {height} radius {radius} subdivision 24 }} }} ] }}")

    def wheel(sign):
        side = "left" if sign > 0 else "right"
        y = (L / 2.0) * sign
        return f"""    HingeJoint {{
      jointParameters HingeJointParameters {{
        axis 0 1 0
        anchor 0 {y} {r}
      }}
      device [
        RotationalMotor {{ name "{side}_motor" maxVelocity {config.MAX_WHEEL_SPEED} maxTorque 5.0 }}
        PositionSensor {{ name "{side}_encoder" }}
      ]
      endPoint Solid {{
        translation 0 {y} {r}
        children [ {cyl_y(r, config.WHEEL_WIDTH, COLORS['wheel'])} ]
        name "{side}_wheel"
        boundingObject Sphere {{ radius {r} subdivision 3 }}
        physics Physics {{ density -1 mass 0.1 }}
      }}
    }}"""

    def caster(c_name, x):
        return f"""    Solid {{
      translation {x} 0 {cr}
      children [ Shape {{ appearance {mat(COLORS['caster'])}
        geometry Sphere {{ radius {cr} subdivision 2 }} }} ]
      name "{c_name}"
      contactMaterial "{CASTER_MATERIAL}"
      boundingObject Sphere {{ radius {cr} subdivision 2 }}
      physics Physics {{ density -1 mass 0.02 }}
    }}"""

    args_str = ""
    if controller_args:
        args_formatted = " ".join([f'"{a}"' for a in controller_args])
        args_str = f"  controllerArgs [ {args_formatted} ]\n"

    children = [
        f"    Pose {{ translation 0 0 {body_z} children [ Shape {{ appearance {mat(body_color)} geometry Box {{ size {bl} {bw} {bh} }} }} ] }}",
        f"""    Lidar {{
      translation 0 0 {config.LIDAR_MOUNT_Z}
      name "{config.LIDAR_NAME}"
      horizontalResolution {config.LIDAR_RAYS}
      fieldOfView {config.LIDAR_FOV:.6f}
      numberOfLayers 1
      verticalFieldOfView 0.1
      minRange {config.LIDAR_MIN_RANGE}
      maxRange {config.LIDAR_MAX_RANGE}
    }}""",
        camera_node(config.CAMERA_NAME, config.CAMERA_MOUNT_X, 0.0, config.CAMERA_FOV),
        camera_node(config.REAR_CAMERA_NAME, config.REAR_CAMERA_MOUNT_X,
                    config.REAR_CAMERA_YAW, config.REAR_CAMERA_FOV),
        f'    GPS {{ name "{config.GPS_NAME}" }}',
        f'    InertialUnit {{ name "{config.IMU_NAME}" }}',
        f'    LED {{ name "{config.LED_NAME}" }}',
        f"""    Pen {{
      translation 0 0 0.001
      name "{config.PEN_NAME}"
      leadSize 0.01
      inkColor 1 0.42 0.05
      inkDensity 1
    }}""",
        f"""    Display {{
      translation 0 0 {config.LIDAR_MOUNT_Z + 0.02}
      name "{config.DISPLAY_NAME}"
      width {config.DISPLAY_WIDTH}
      height {config.DISPLAY_HEIGHT}
    }}""",
        f"""    TouchSensor {{
      translation {bl / 2.0 + 0.01} 0 {body_z}
      children [ Shape {{ appearance {mat(COLORS['bumper'])} geometry Box {{ size 0.02 {bw * 0.9} {bh * 0.6} }} }} ]
      name "bumper_f"
      boundingObject Box {{ size 0.02 {bw * 0.9} {bh * 0.6} }}
      physics Physics {{ density -1 mass 0.02 }}
      type "bumper"
    }}""",
        f"""    TouchSensor {{
      translation {-bl / 2.0 - 0.01} 0 {body_z}
      children [ Shape {{ appearance {mat(COLORS['bumper'])} geometry Box {{ size 0.02 {bw * 0.9} {bh * 0.6} }} }} ]
      name "bumper_r"
      boundingObject Box {{ size 0.02 {bw * 0.9} {bh * 0.6} }}
      physics Physics {{ density -1 mass 0.02 }}
      type "bumper"
    }}""",
        '    Emitter { name "emitter" channel 1 }',
        '    Receiver { name "receiver" channel 1 }',
        caster(f"{def_name.lower()}_caster_f", caster_x),
        caster(f"{def_name.lower()}_caster_r", -caster_x),
        wheel(+1),
        wheel(-1),
    ]

    robot_bounds = f"Pose {{ translation 0 0 {body_z} children [ Box {{ size {bl} {bw} {bh} }} ] }}"
    physics = f"Physics {{ density -1 mass {body_mass} }}"
    if with_arm:
        # rear support post of the arm (part of the base) + the arm itself
        post_h = config.ARM_SHOULDER_Z - (cr + bh)
        post_c = (config.ARM_BASE_X, 0.0, cr + bh + post_h / 2.0)
        post_s = (config.ARM_POST_SIZE, config.ARM_POST_SIZE, post_h)
        children.append("    " + _box(post_s, COLORS["steel"], post_c))
        children.append(pitch_arm_vrml())
        robot_bounds = (f"Group {{ children [ {robot_bounds} {_bbox(post_s, post_c)} ] }}")
        # counterweight folded into the base's mass and centre of mass
        m = body_mass + config.ARM_COUNTERWEIGHT_MASS
        com_x = config.ARM_COUNTERWEIGHT_MASS * config.ARM_COUNTERWEIGHT_X / m
        physics = (f"Physics {{ density -1 mass {f(m)} "
                   f"centerOfMass [ {vec(com_x, 0, body_z)} ] }}")

    # Mobile manipulator 2R arm (Phase 14)
    if has_arm:
        arm_vrml = f"""    SliderJoint {{
      jointParameters JointParameters {{
        axis 0 0 1
        minStop 0.0
        maxStop 0.70
      }}
      device [
        LinearMotor {{ name "lift_motor" maxVelocity 0.15 maxForce 200 }}
        PositionSensor {{ name "lift_sensor" }}
      ]
      endPoint Solid {{
        translation 0.10 0 {body_z + 0.12}
        children [
          Shape {{ appearance {mat(COLORS['steel'])} geometry Box {{ size 0.12 0.12 0.08 }} }}
          HingeJoint {{
            jointParameters HingeJointParameters {{
              axis 0 0 1
              minStop -2.618
              maxStop 2.618
            }}
            device [
              RotationalMotor {{ name "arm_motor_1" maxVelocity 1.5 maxTorque 30 }}
              PositionSensor {{ name "arm_sensor_1" }}
            ]
            endPoint Solid {{
              translation 0.175 0 0.04
              children [
                Pose {{ translation 0 0 0 children [ Shape {{ appearance {mat(COLORS['steel'])} geometry Box {{ size 0.35 0.06 0.04 }} }} ] }}
                HingeJoint {{
                  jointParameters HingeJointParameters {{
                    axis 0 0 1
                    anchor 0.175 0 0
                    minStop -2.792
                    maxStop 2.792
                  }}
                  device [
                    RotationalMotor {{ name "arm_motor_2" maxVelocity 1.5 maxTorque 30 }}
                    PositionSensor {{ name "arm_sensor_2" }}
                  ]
                  endPoint Solid {{
                    translation 0.175 0 0
                    children [
                      Pose {{ translation 0.15 0 0 children [ Shape {{ appearance {mat(COLORS['steel'])} geometry Box {{ size 0.30 0.05 0.04 }} }} ] }}
                      HingeJoint {{
                        jointParameters HingeJointParameters {{
                          axis 0 0 1
                          anchor 0.30 0 0
                          minStop -1.5708
                          maxStop 1.5708
                        }}
                        device [
                          RotationalMotor {{ name "wrist_motor" maxVelocity 2.0 maxTorque 10 }}
                          PositionSensor {{ name "wrist_sensor" }}
                        ]
                        endPoint Solid {{
                          translation 0.30 0 0
                          children [
                            Shape {{ appearance Appearance {{ material Material {{ diffuseColor 0.25 0.25 0.25 }} }} geometry Box {{ size 0.06 0.14 0.05 }} }}
                            DistanceSensor {{
                              translation 0.03 0 0
                              name "palm_ir"
                              lookupTable [ 0 1024 0, 0.25 0 0 ]
                              type "infra-red"
                            }}
                            SliderJoint {{
                              jointParameters JointParameters {{ axis 0 1 0 minStop 0.0 maxStop 0.13 }}
                              device [
                                LinearMotor {{ name "finger_motor_1" maxVelocity 0.1 maxForce 15 }}
                                PositionSensor {{ name "finger_sensor_1" }}
                              ]
                              endPoint Solid {{
                                translation 0.06 0.05 0
                                children [
                                  Shape {{ appearance {mat(COLORS['gripper_red'])} geometry Box {{ size 0.08 0.02 0.04 }} }}
                                  TouchSensor {{
                                    name "finger_touch_1"
                                    type "force"
                                    boundingObject Box {{ size 0.08 0.02 0.04 }}
                                    physics Physics {{ density -1 mass 0.05 }}
                                  }}
                                ]
                                boundingObject Box {{ size 0.08 0.02 0.04 }}
                                physics Physics {{ density -1 mass 0.05 }}
                              }}
                            }}
                            SliderJoint {{
                              jointParameters JointParameters {{ axis 0 -1 0 minStop 0.0 maxStop 0.13 }}
                              device [
                                LinearMotor {{ name "finger_motor_2" maxVelocity 0.1 maxForce 15 }}
                                PositionSensor {{ name "finger_sensor_2" }}
                              ]
                              endPoint Solid {{
                                translation 0.06 -0.05 0
                                children [
                                  Shape {{ appearance {mat(COLORS['gripper_red'])} geometry Box {{ size 0.08 0.02 0.04 }} }}
                                  TouchSensor {{
                                    name "finger_touch_2"
                                    type "force"
                                    boundingObject Box {{ size 0.08 0.02 0.04 }}
                                    physics Physics {{ density -1 mass 0.05 }}
                                  }}
                                ]
                                boundingObject Box {{ size 0.08 0.02 0.04 }}
                                physics Physics {{ density -1 mass 0.05 }}
                              }}
                            }}
                          ]
                          boundingObject Box {{ size 0.06 0.14 0.05 }}
                          physics Physics {{ density -1 mass 0.2 }}
                        }}
                      }}
                    ]
                    boundingObject Pose {{ translation 0.15 0 0 children [ Box {{ size 0.30 0.05 0.04 }} ] }}
                    physics Physics {{ density -1 mass 0.4 }}
                  }}
                }}
              ]
              boundingObject Box {{ size 0.35 0.06 0.04 }}
              physics Physics {{ density -1 mass 0.5 }}
            }}
          }}
        ]
        boundingObject Box {{ size 0.12 0.12 0.08 }}
        physics Physics {{ density -1 mass 0.8 }}
      }}
    }}"""
        children.append(arm_vrml)

    children_str = "\n".join(children)

    return f"""DEF {def_name} Robot {{
  translation {start_pos[0]} {start_pos[1]} {start_pos[2]}
  rotation {start_rot[0]} {start_rot[1]} {start_rot[2]} {start_rot[3]}
{args_str}  children [
{children_str}
  ]
  name "{robot_name}"
  boundingObject {robot_bounds}
  physics {physics}
  controller "{controller}"
  supervisor TRUE
}}"""


def build_world(controller):
    w = []
    w.append("#VRML_SIM R2025a utf8")
    w.append("")
    w.append("# GENERATED by tools/generate_world.py from config.py. DO NOT EDIT BY HAND.")
    w.append("# Regenerate with: python tools/generate_world.py")
    w.append("")
    w.append("WorldInfo {\n"
             "  info [ \"IntelliBot warehouse (generated)\" ]\n"
             "  title \"IntelliWarehouse\"\n"
             f"  basicTimeStep {config.BASIC_TIME_STEP_MS}\n"
             "  coordinateSystem \"ENU\"\n"
             "  contactProperties [\n"
             f"    ContactProperties {{ material2 \"{CASTER_MATERIAL}\" coulombFriction [ 0 ] }}\n"
             f"    ContactProperties {{ material1 \"{PAD_MATERIAL}\" material2 \"{PARCEL_MATERIAL}\" coulombFriction [ 1.2 ] }}\n"
             "  ]\n"
             "}")
    w.append(f"DEF VIEWPOINT Viewpoint {{ orientation {VIEW_ANGLED[1]} "
             f"position {VIEW_ANGLED[0]} }}")
    w.append("DEF BACKGROUND Background { skyColor [ 0.75 0.8 0.85 ] luminosity 1 }")
    # Fault injection (fault_injector.py): smoke = Webots Fog. visibilityRange 0
    # disables it; the supervisor lowers it at run time (cameras only — the
    # LiDAR attenuation is modelled in software by the injector).
    w.append(f"DEF {getattr(config, 'SMOKE_FOG_DEF', 'SMOKE')} Fog {{ color 0.62 0.62 0.62 "
             f"fogType \"EXPONENTIAL2\" visibilityRange 0 }}")
    # flat, even lighting; NO shadows to keep HSV detection stable. Two
    # opposed directional lights so EVERY vertical face (+-x, +-y) is lit by
    # one of them: with a single light the package faces pointing at the
    # robot got ambient only (HSV value ~46 < detector minimum) and FIND_PACKAGE
    # never saw them.
    for i, d in enumerate(("0.3 -0.4 -1", "-0.3 0.4 -1"), start=1):
        w.append(f"DEF DIRLIGHT_{i} DirectionalLight {{ direction {d} "
                 f"intensity 0.8 ambientIntensity 0.5 castShadows FALSE }}")
    w.append("")

    # floor (top face at z = 0) — collides, everything rests on it
    t = config.WALL_THICKNESS
    fw = (config.ARENA_X_MAX - config.ARENA_X_MIN) + 2 * t
    fh = (config.ARENA_Y_MAX - config.ARENA_Y_MIN) + 2 * t
    w.append(solid("FLOOR", "0 0 -0.025", f"{fw:g} {fh:g} 0.05", COLORS["floor"]))
    # walls — the SAME rectangles the occupancy grid uses (config._WALLS)
    wh = config.WALL_HEIGHT
    for (cx, cy, sx, sy), d in zip(config._WALLS,
                                   ("WALL_N", "WALL_S", "WALL_E", "WALL_W")):
        w.append(solid(d, f"{cx:g} {cy:g} {wh / 2:g}", f"{sx:g} {sy:g} {wh:g}",
                       COLORS["wall"]))

    # shelves + crates (static: no physics)
    for x, y, sx, sy, h, d in config.SHELVES:
        w.append(solid(d, f"{x} {y} {h / 2}", f"{sx} {sy} {h}", COLORS["shelf"]))
    for x, y, sx, sy, h, d in config.CRATES:
        w.append(solid(d, f"{x} {y} {h / 2}", f"{sx} {sy} {h}", COLORS["crate"]))

    # small varied-shape obstacles (cylinders / spheres / small crates) —
    # they force A* to weave; grid gets their bounding squares from config
    for x, y, r, h, d in config.CYLINDERS:
        w.append(f'DEF {d} Solid {{\n'
                 f'  translation {x} {y} {h / 2}\n'
                 f'  children [ Shape {{ appearance {mat(COLORS["barrel"])} '
                 f'geometry Cylinder {{ radius {r} height {h} subdivision 20 }} }} ]\n'
                 f'  name "{d.lower()}"\n'
                 f'  boundingObject Cylinder {{ radius {r} height {h} subdivision 20 }}\n'
                 f'  physics Physics {{ density -1 mass 1.0 }}\n'
                 f'}}\n')
    for x, y, r, d in config.SPHERES:
        w.append(f'DEF {d} Solid {{\n'
                 f'  translation {x} {y} {r}\n'
                 f'  children [ Shape {{ appearance {mat(COLORS["ball"])} '
                 f'geometry Sphere {{ radius {r} subdivision 4 }} }} ]\n'
                 f'  name "{d.lower()}"\n'
                 f'  boundingObject Sphere {{ radius {r} subdivision 4 }}\n'
                 f'  physics Physics {{ density -1 mass 0.8 }}\n'
                 f'}}\n')
    for x, y, sz, h, d in config.CRATES_EXTRA:
        w.append(solid(d, f"{x} {y} {h / 2}", f"{sz} {sz} {h}", COLORS["crate"]))

    # floor patches — NO boundingObject (non-colliding)
    for d, (px, py, psx, psy), c in (("PICKUP_AREA", config.PICKUP_AREA, "pickup"),
                                     ("ZONE_A", config.ZONE_A, "zone_a"),
                                     ("ZONE_B", config.ZONE_B, "zone_b")):
        w.append(solid(d, f"{px} {py} 0.005", f"{psx} {psy} 0.01",
                       COLORS[c], bounding=False))

    # packages — pushable Solids with physics (+ model/recognitionColors for
    # the Recognition node: Webots only recognizes Solids with the
    # recognitionColors field; model is the reported label)
    s = config.PKG_SIZE
    pkg_rgb = {"red": "1 0 0", "green": "0 1 0", "blue": "0 0 1"}
    for defname, x, y, color in config.PACKAGES:
        body = solid(defname, f"{x} {y} {s / 2}", f"{s} {s} {s}",
                     COLORS[color],
                     physics=f"Physics {{ density -1 mass {f(config.PKG_MASS)} }}")
        body = body.replace(
            f'name "{defname.lower()}"',
            'model "%s"\n  recognitionColors [ %s ]\n  name "%s"\n  contactMaterial "%s"'
            % (defname, pkg_rgb[color], defname.lower(), PARCEL_MATERIAL))
        # passive latches (Connector) on the 4 vertical faces: any face the
        # gripper approaches can be latched
        body = body.replace("  ]\n", "\n" + parcel_connectors() + "\n  ]\n", 1)
        w.append(body)

    # destination racks (one per dispatch zone, against the east wall)
    for rack in shelving.build_racks().values():
        w.append(build_rack(rack))

    # final-eval hook: dynamic obstacle, parked in a free corner off every route
    dx, dy = config.DYN_OBSTACLE_PARK
    w.append(solid(config.DYN_OBSTACLE_DEF, f"{dx} {dy} 0.2", "0.4 0.4 0.4",
                   COLORS["crate"], physics="Physics { density -1 mass 5 }"))

    # Moving obstacles, driven KINEMATICALLY by the supervisor
    # (moving_obstacles.py): no Physics node, so they can neither be shoved off
    # their lanes nor fall asleep in ODE, and never push the robot with force.
    c1 = config.MOVING_OBSTACLE_LANES["patrol_north"][0]
    c2 = config.MOVING_OBSTACLE_LANES["cross_aisle_1"][0]
    w.append(solid("CART_1", f"{c1[0]} {c1[1]} 0.15", "0.45 0.35 0.3", COLORS["cart_1"]))
    w.append(solid("CART_2", f"{c2[0]} {c2[1]} 0.15", "0.4 0.4 0.3", COLORS["cart_2"]))
    # yellow look-alike robot (replaces the grey FAST_SPHERE; the brown
    # FAST_CART that also crossed the transit lane was removed)
    w.append(build_decoy())

    w.append(build_robot(controller, with_arm=True))
    w.append("")
    return "\n".join(w)


def build_world_v2(controller="intellibot_controller"):
    """Phase 13: Build full 12x8m Warehouse v2 world."""
    w = [
        "#VRML_SIM R2025a utf8",
        "WorldInfo {",
        "  basicTimeStep 16",
        "  contactProperties [",
        "    ContactProperties {",
        '      material1 "caster"',
        '      material2 "floor"',
        "      coulombFriction [ 0 ]",
        "    }",
        "    ContactProperties {",
        '      material1 "box"',
        '      material2 "box"',
        "      coulombFriction [ 1.2 ]",
        "    }",
        "    ContactProperties {",
        '      material1 "box"',
        '      material2 "floor"',
        "      coulombFriction [ 0.8 ]",
        "    }",
        "    ContactProperties {",
        '      material1 "gripper"',
        '      material2 "box"',
        "      coulombFriction [ 1.5 ]",
        "    }",
        "  ]",
        '  coordinateSystem "ENU"',  # was "NUE" (y-up) but all data is z-up
        "}",
        "Viewpoint {",
        "  orientation 1 0 0 0.85",
        "  position 0 -11 12",
        "}",
        # Multiple controllable lights (Phase 10.4 / 13.1)
        "DEF LIGHT_1 PointLight {",
        "  attenuation 0 0 1",
        "  color 1 1 0.95",
        "  intensity 1.5",
        "  location -4.0 0 3.5",
        "  castShadows TRUE",
        "}",
        "DEF LIGHT_2 PointLight {",
        "  attenuation 0 0 1",
        "  color 1 1 0.95",
        "  intensity 1.5",
        "  location 0.0 0 3.5",
        "  castShadows TRUE",
        "}",
        "DEF LIGHT_3 PointLight {",
        "  attenuation 0 0 1",
        "  color 1 1 0.95",
        "  intensity 1.5",
        "  location 4.0 0 3.5",
        "  castShadows TRUE",
        "}",
        # 12x8m floor
        "Solid {",
        "  translation 0 0 -0.01",
        "  children [ Shape { appearance Appearance { material Material { diffuseColor 0.55 0.53 0.50 } } geometry Box { size 12.0 8.0 0.02 } } ]",
        '  name "floor"',
        "  boundingObject Box { size 12.0 8.0 0.02 }",
        '  contactMaterial "floor"',
        "}",
        # Perimeter walls (12x8m arena, height 1.5m)
        'Solid { translation 0 4.05 0.75 children [ Shape { appearance Appearance { material Material { diffuseColor 0.75 0.75 0.75 } } geometry Box { size 12.2 0.1 1.5 } } ] name "wall_north" boundingObject Box { size 12.2 0.1 1.5 } }',
        'Solid { translation 0 -4.05 0.75 children [ Shape { appearance Appearance { material Material { diffuseColor 0.75 0.75 0.75 } } geometry Box { size 12.2 0.1 1.5 } } ] name "wall_south" boundingObject Box { size 12.2 0.1 1.5 } }',
        'Solid { translation 6.05 0 0.75 children [ Shape { appearance Appearance { material Material { diffuseColor 0.75 0.75 0.75 } } geometry Box { size 0.1 8.2 1.5 } } ] name "wall_east" boundingObject Box { size 0.1 8.2 1.5 } }',
        'Solid { translation -6.05 0 0.75 children [ Shape { appearance Appearance { material Material { diffuseColor 0.75 0.75 0.75 } } geometry Box { size 0.1 8.2 1.5 } } ] name "wall_west" boundingObject Box { size 0.1 8.2 1.5 } }',
        # Dispatch zones A (yellow) and B (cyan)
        'Solid { translation 4.8 2.0 0.001 children [ Shape { appearance Appearance { material Material { diffuseColor 0.95 0.9 0.1 } } geometry Box { size 1.2 1.2 0.002 } } ] name "zone_a" }',
        'Solid { translation 4.8 -2.0 0.001 children [ Shape { appearance Appearance { material Material { diffuseColor 0.1 0.85 0.9 } } geometry Box { size 1.2 1.2 0.002 } } ] name "zone_b" }',
        # Docks
        'Solid { translation -4.8 2.5 0.001 children [ Shape { appearance Appearance { material Material { diffuseColor 0.35 0.50 0.70 } } geometry Box { size 0.8 0.8 0.002 } } ] name "dock_1" }',
        'Solid { translation -4.8 -2.5 0.001 children [ Shape { appearance Appearance { material Material { diffuseColor 0.35 0.50 0.70 } } geometry Box { size 0.8 0.8 0.002 } } ] name "dock_2" }',
        'Solid { translation -4.8 -1.0 0.001 children [ Shape { appearance Appearance { material Material { diffuseColor 0.35 0.50 0.70 } } geometry Box { size 0.8 0.8 0.002 } } ] name "dock_3" }',
        # Receiving pallet
        'Solid { translation -4.5 0.0 0.025 children [ Shape { appearance Appearance { material Material { diffuseColor 0.85 0.80 0.70 } } geometry Box { size 1.4 1.0 0.05 } } ] name "pallet" boundingObject Box { size 1.4 1.0 0.05 } contactMaterial "floor" }',
    ]

    # Landmarks (6 posts)
    for idx, (lx, ly) in enumerate(config.V2_LANDMARKS, 1):
        w.append(f'Solid {{ translation {lx} {ly} 0.6 children [ Shape {{ appearance Appearance {{ material Material {{ diffuseColor 0.95 0.95 0.95 }} }} geometry Cylinder {{ height 1.2 radius 0.05 }} }} ] name "landmark_{idx}" boundingObject Cylinder {{ height 1.2 radius 0.05 }} }}')

    # 4 Shelf units
    for s_id, s_info in config.V2_SHELVES.items():
        sx, sy, sz = s_info['pos']
        # 3 horizontal tiers
        for lvl, l_z in enumerate([0.15, 0.55, 0.95], 1):
            w.append(f'Solid {{ translation {sx} {sy} {l_z} children [ Shape {{ appearance Appearance {{ material Material {{ diffuseColor 0.22 0.24 0.28 }} }} geometry Box {{ size 2.4 0.6 0.04 }} }} ] name "{s_id}_L{lvl}" boundingObject Box {{ size 2.4 0.6 0.04 }} }}')
        # Shelf back wall (for IR occupancy check)
        by = sy + (0.28 if sy > 0 else -0.28)
        w.append(f'Solid {{ translation {sx} {by} 0.6 children [ Shape {{ appearance Appearance {{ material Material {{ diffuseColor 0.25 0.27 0.32 }} }} geometry Box {{ size 2.4 0.04 1.2 }} }} ] name "{s_id}_back" boundingObject Box {{ size 2.4 0.04 1.2 }} }}')

    # Receiving pile: 10 boxes in 2 layers
    box_colors = ['red', 'green', 'blue', 'red', 'green', 'blue', 'red', 'green', 'blue', 'red']
    rgb_map = {'red': '0.9 0.1 0.1', 'green': '0.1 0.8 0.1', 'blue': '0.1 0.2 0.9'}
    l1_coords = [
        (-4.75, -0.25, 0.16), (-4.50, -0.25, 0.16), (-4.25, -0.25, 0.16),
        (-4.75, 0.25, 0.16), (-4.50, 0.25, 0.16), (-4.25, 0.25, 0.16)
    ]
    l2_coords = [
        (-4.62, -0.12, 0.38), (-4.38, -0.12, 0.38),
        (-4.62, 0.12, 0.38), (-4.38, 0.12, 0.38)
    ]
    all_coords = l1_coords + l2_coords
    for idx, (bx, by, bz) in enumerate(all_coords, 1):
        c_name = box_colors[idx - 1]
        c_rgb = rgb_map[c_name]
        w.append(f'''DEF PKG_{idx} Solid {{
  translation {bx:.2f} {by:.2f} {bz:.2f}
  children [
    Shape {{
      appearance Appearance {{
        material Material {{ diffuseColor {c_rgb} }}
        texture ImageTexture {{ url [ "textures/aruco/marker_{idx}.png" ] }}
      }}
      geometry Box {{ size 0.22 0.22 0.22 }}
    }}
  ]
  name "pkg_{idx}"
  model "PKG_{idx}"
  recognitionColors [ {c_rgb} ]
  boundingObject Box {{ size 0.22 0.22 0.22 }}
  physics Physics {{ density -1 mass 0.4 }}
  contactMaterial "box"
}}''')

    # Moving obstacles (Supervisor-driven)
    w.append(solid("CART_1", "-4.0 3.2 0.2", "0.5 0.4 0.35",
                   COLORS["cart_1"], physics="Physics { density -1 mass 5.0 }"))
    w.append(solid("CART_2", "0.5 -3.2 0.2", "0.45 0.45 0.35",
                   COLORS["cart_2"], physics="Physics { density -1 mass 5.0 }"))

    # Dual Mobile Manipulator Robots (Phase 14 & Phase 15.1)
    w.append(build_robot(controller, def_name="ROBOT_1", robot_name="ROBOT_1",
                         start_pos=(-4.8, 2.5, 0.0), start_rot=(0, 0, 1, 0.0),
                         controller_args=["R1"], body_color=COLORS["robot_1"], has_arm=True))
    w.append(build_robot(controller, def_name="ROBOT_2", robot_name="ROBOT_2",
                         start_pos=(-4.8, -2.5, 0.0), start_rot=(0, 0, 1, 0.0),
                         controller_args=["R2"], body_color=COLORS["robot_2"], has_arm=True))

    # Fleet Manager Supervisor (Phase 15.2)
    w.append("""DEF FLEET_MANAGER Robot {
  translation -5.5 0 0.5
  children [
    Shape {
      appearance Appearance { material Material { diffuseColor 0.2 0.2 0.2 transparency 0.8 } }
      geometry Box { size 0.1 0.1 0.1 }
    }
    Emitter {
      name "emitter"
      channel 1
    }
    Receiver {
      name "receiver"
      channel 1
    }
  ]
  name "FLEET_MANAGER"
  controller "fleet_manager"
  supervisor TRUE
}""")
    w.append("")
    return "\n".join(w)


def main():
    os.makedirs(WORLDS_DIR, exist_ok=True)
    out_dir = os.path.normpath(os.path.join(WORLDS_DIR, "..", "worlds_output"))
    os.makedirs(out_dir, exist_ok=True)
    for fname, controller in WORLDS.items():
        if "v2" in fname:
            content = build_world_v2(controller)
        else:
            content = build_world(controller)
        out_path = os.path.normpath(os.path.join(WORLDS_DIR, fname))
        with open(out_path, "w", newline="\n") as f:
            f.write(content)
        with open(os.path.join(out_dir, fname), "w", newline="\n") as f:
            f.write(content)
        print(f"wrote {out_path} ({os.path.getsize(out_path)} bytes, "
              f"controller \"{controller}\") + copy in worlds_output/")


if __name__ == "__main__":
    main()
