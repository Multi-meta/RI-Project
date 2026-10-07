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

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "controllers", "intellibot_controller"))
import config  # noqa: E402

WORLDS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                          "worlds")
WORLDS = {
    "warehouse.wbt": "intellibot_controller",
    "warehouse_drive_test.wbt": "drive_test",
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
    "wheel": (0.1, 0.1, 0.1),
    "caster": (0.3, 0.3, 0.3),
    "sensor": (0.05, 0.05, 0.05),
    "barrel": (0.75, 0.4, 0.05),
    "ball": (0.45, 0.45, 0.50),  # DESATURATED gray-blue: must stay outside every HSV package window (a saturated blue ball was mistaken for package P3!)
}

CASTER_MATERIAL = "caster"          # zero-friction contact material


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


def build_robot(controller):
    """DEF INTELLIBOT — differential-drive robot per project_summary §8.

    Support: 2 driven wheels on the center axle + 2 passive caster spheres
    (front/rear) whose contact material has zero friction (WorldInfo
    contactProperties), so the robot rests on 4 points but only the wheels
    produce traction.
    """
    r, L = config.WHEEL_RADIUS, config.WHEEL_SEPARATION
    bl, bw, bh = config.BODY_LENGTH, config.BODY_WIDTH, config.BODY_HEIGHT
    cr = config.CASTER_RADIUS
    body_z = cr + bh / 2.0          # body bottom flush with caster centers
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
        # collision = sphere of the wheel radius: ONE contact point exactly at
        # y = +-L/2. A cylinder contacts the floor at its outer rim, which made
        # the effective track width L + WHEEL_WIDTH (measured -14 % yaw rate).
        boundingObject Sphere {{ radius {r} subdivision 3 }}
        physics Physics {{ density -1 mass 0.1 }}
      }}
    }}"""

    def caster(name, x):
        return f"""    Solid {{
      translation {x} 0 {cr}
      children [ Shape {{ appearance {mat(COLORS['caster'])}
        geometry Sphere {{ radius {cr} subdivision 2 }} }} ]
      name "{name}"
      contactMaterial "{CASTER_MATERIAL}"
      boundingObject Sphere {{ radius {cr} subdivision 2 }}
      physics Physics {{ density -1 mass 0.02 }}
    }}"""

    cam_vis = (f"Pose {{ translation -0.01 0 0 children [ Shape {{ appearance "
               f"{mat(COLORS['sensor'])} geometry Box {{ size 0.02 0.04 0.03 }} }} ] }}")

    return f"""DEF INTELLIBOT Robot {{
  translation {config.ROBOT_START[0]} {config.ROBOT_START[1]} 0
  rotation 0 0 1 {config.ROBOT_START[2]}
  children [
    Pose {{ translation 0 0 {body_z}
      children [ Shape {{ appearance {mat(COLORS['body'])} geometry Box {{ size {bl} {bw} {bh} }} }} ] }}
    Lidar {{
      translation 0 0 {config.LIDAR_MOUNT_Z}
      name "{config.LIDAR_NAME}"
      horizontalResolution {config.LIDAR_RAYS}
      fieldOfView {config.LIDAR_FOV:.6f}
      numberOfLayers 1
      verticalFieldOfView 0.1
      minRange {config.LIDAR_MIN_RANGE}
      maxRange {config.LIDAR_MAX_RANGE}
    }}
    Camera {{
      translation {config.CAMERA_MOUNT_X} 0 {config.CAMERA_MOUNT_Z}
      children [ {cam_vis} ]
      name "{config.CAMERA_NAME}"
      fieldOfView {config.CAMERA_FOV}
      width {config.CAMERA_WIDTH}
      height {config.CAMERA_HEIGHT}
    }}
    GPS {{ name "{config.GPS_NAME}" }}
    InertialUnit {{ name "{config.IMU_NAME}" }}
    LED {{ name "{config.LED_NAME}" }}
    Display {{
      translation 0 0 {config.LIDAR_MOUNT_Z + 0.02}
      name "{config.DISPLAY_NAME}"
      width {config.DISPLAY_WIDTH}
      height {config.DISPLAY_HEIGHT}
    }}
{caster("caster_front", caster_x)}
{caster("caster_rear", -caster_x)}
{wheel(+1)}
{wheel(-1)}
  ]
  name "INTELLIBOT"
  boundingObject Pose {{ translation 0 0 {body_z} children [
    Box {{ size {bl} {bw} {bh} }} ] }}
  physics Physics {{ density -1 mass {config.BODY_MASS} }}
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
             "  ]\n"
             "}")
    w.append(f"DEF VIEWPOINT Viewpoint {{ orientation {VIEW_ANGLED[1]} "
             f"position {VIEW_ANGLED[0]} }}")
    w.append("Background { skyColor [ 0.75 0.8 0.85 ] }")
    # flat, even lighting; NO shadows to keep HSV detection stable. Two
    # opposed directional lights so EVERY vertical face (+-x, +-y) is lit by
    # one of them: with a single light the package faces pointing at the
    # robot got ambient only (HSV value ~46 < detector minimum) and FIND_PACKAGE
    # never saw them.
    for d in ("0.3 -0.4 -1", "-0.3 0.4 -1"):
        w.append(f"DirectionalLight {{ direction {d} intensity 0.8 "
                 f"ambientIntensity 0.5 castShadows FALSE }}")
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

    # packages — pushable Solids with physics
    s = config.PKG_SIZE
    for defname, x, y, color in config.PACKAGES:
        w.append(solid(defname, f"{x} {y} {s / 2}", f"{s} {s} {s}",
                       COLORS[color], physics="Physics { density -1 mass 0.5 }"))

    # final-eval hook: dynamic obstacle, parked in a free corner off every route
    dx, dy = config.DYN_OBSTACLE_PARK
    w.append(solid(config.DYN_OBSTACLE_DEF, f"{dx} {dy} 0.2", "0.4 0.4 0.4",
                   COLORS["crate"], physics="Physics { density -1 mass 5 }"))

    w.append(build_robot(controller))
    w.append("")
    return "\n".join(w)


def main():
    os.makedirs(WORLDS_DIR, exist_ok=True)
    # worlds_output/ keeps plain copies of the latest generated worlds
    # (staging dir; worlds/ is what Webots opens).
    out_dir = os.path.normpath(os.path.join(WORLDS_DIR, "..", "worlds_output"))
    os.makedirs(out_dir, exist_ok=True)
    for fname, controller in WORLDS.items():
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
