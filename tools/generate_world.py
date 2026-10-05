"""generate_world.py — write worlds/warehouse.wbt from config.py.

The world is GENERATED so the simulated world and the occupancy grid can
never disagree (project_summary.md rule 3). Everything is built from
primitive nodes (Solid / Shape / Box / Cylinder / Sphere) to avoid
EXTERNPROTO dependencies across Webots versions.

Run:  python tools/generate_world.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "controllers", "intellibot_controller"))
import config  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                   "worlds", "warehouse.wbt")

# BGR not needed here; these are Webots RGB 0..1
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
}


def mat(color, name=None):
    def_name = f" DEF {name}" if name else ""
    return (f"Appearance{{{def_name} material Material{{diffuseColor "
            f"{color[0]:g} {color[1]:g} {color[2]:g} }}}}")


def solid(defname, translation, size, color, physics=None,
          bounding=True, rotation=None, name=None):
    """Axis-aligned box Solid."""
    s = f"DEF {defname} Solid {{\n"
    s += f"  translation {translation}\n"
    if rotation:
        s += f"  rotation {rotation}\n"
    if name:
        s += f"  name \"{name}\"\n"
    s += ("  children [\n"
          f"    Shape {{ appearance {mat(color)} "
          f"geometry Box {{ size {size} }} }}\n"
          "  ]\n")
    if bounding:
        s += f"  boundingObject Box {{ size {size} }}\n"
    if physics:
        s += f"  physics {physics}\n"
    s += "}\n"
    return s


def build_robot():
    """DEF INTELLIBOT — differential-drive robot per project_summary §8."""
    r, L = config.WHEEL_RADIUS, config.WHEEL_SEPARATION
    bl, bw, bh = config.BODY_LENGTH, config.BODY_WIDTH, config.BODY_HEIGHT
    cr = config.CASTER_RADIUS
    # body bottom sits above ground; wheels (r) and casters (cr) touch z=0
    body_z = r + 0.03
    wheel_y = L / 2.0
    wheel_x = 0.0
    caster_x = bl / 2.0 - 0.06

    max_vel = config.MAX_WHEEL_SPEED
    wheel_geo = (f"Transform {{ rotation 1 0 0 1.5708 children [ "
                 f"Shape {{ appearance {mat(COLORS['wheel'])} geometry "
                 f"Cylinder {{ height {config.WHEEL_WIDTH} radius {r} "
                 f"subdivision 24 }} }} ] }}")

    def wheel(side, sign):
        y = wheel_y * sign
        return f"""
    HingeJoint {{
      jointParameters HingeJointParameters {{
        axis 0 1 0
        anchor {wheel_x} {y} {r}
      }}
      device [
        RotationalMotor {{ name "{ 'left_motor' if sign > 0 else 'right_motor' }"
          maxVelocity {max_vel} maxTorque 5.0 }}
        PositionSensor {{ name "{ 'left_encoder' if sign > 0 else 'right_encoder' }" }}
      ]
      endPoint Solid {{
        translation {wheel_x} {y} {r}
        children [ {wheel_geo} ]
        name "{ 'left_wheel' if sign > 0 else 'right_wheel' }"
        boundingObject Transform {{ rotation 1 0 0 1.5708 children [
          Cylinder {{ height {config.WHEEL_WIDTH} radius {r} subdivision 24 }} ] }}
        physics Physics {{ density -1 mass 0.1 }}
      }}
    }}"""

    return f"""DEF INTELLIBOT Robot {{
  translation {config.ROBOT_START[0]} {config.ROBOT_START[1]} 0
  rotation 0 0 1 {config.ROBOT_START[2]}
  supervisor TRUE
  controller "intellibot_controller"
  children [
    Transform {{ translation 0 0 {body_z}
      children [ Shape {{ appearance {mat(COLORS['body'])} geometry Box {{ size {bl} {bw} {bh} }} }} ] }}
    DEF LIDAR_MOUNT Transform {{
      translation 0 0 {config.LIDAR_MOUNT_Z}
      children [
        Lidar {{
          name "{config.LIDAR_NAME}"
          rotation 0 1 0 0
          horizontalResolution {config.LIDAR_RAYS}
          numberOfLayers 1
          minRange {config.LIDAR_MIN_RANGE}
          maxRange {config.LIDAR_MAX_RANGE}
          fieldOfView {config.LIDAR_FOV}
        }}
      ]
    }}
    DEF CAM_MOUNT Transform {{
      translation {config.CAMERA_MOUNT_X} 0 {config.CAMERA_MOUNT_Z}
      children [
        Camera {{
          name "{config.CAMERA_NAME}"
          rotation 0 1 0 0
          width {config.CAMERA_WIDTH}
          height {config.CAMERA_HEIGHT}
          fieldOfView {config.CAMERA_FOV}
        }}
      ]
    }}
    GPS {{ name "{config.GPS_NAME}" }}
    InertialUnit {{ name "{config.IMU_NAME}" }}
    DEF CASTER_FRONT Transform {{
      translation {caster_x} 0 {cr}
      children [ Shape {{ appearance {mat(COLORS['caster'])}
        geometry Sphere {{ radius {cr} subdivision 12 }} }} ]
    }}
    DEF CASTER_REAR Transform {{
      translation {-caster_x} 0 {cr}
      children [ Shape {{ appearance {mat(COLORS['caster'])}
        geometry Sphere {{ radius {cr} subdivision 12 }} }} ]
    }}
{wheel('left', +1)},
{wheel('right', -1)}
  ]
  name "INTELLIBOT"
  boundingObject Transform {{ translation 0 0 {body_z} children [
    Box {{ size {bl} {bw} {bh} }} ] }}
  physics Physics {{ density -1 mass {config.BODY_MASS}
    contactProperties [ ContactProperties {{ coulombFriction [ 0, 0 ] }} ] }}
}}"""


def main():
    w = []
    w.append("#VRML_SIM R2025a")
    w.append("")
    w.append("# warehouse.wbt — GENERATED by tools/generate_world.py. DO NOT EDIT BY HAND.")
    w.append("# Regenerate with: python tools/generate_world.py")
    w.append("")
    w.append(f"WorldInfo {{ coordinateSystem \"ENU\" basicTimeStep "
             f"{config.BASIC_TIME_STEP_MS} contactProperties [] }}")
    w.append("Viewpoint { orientation -0.35 0.75 0.55 4.4 position -7 6 7 }")
    # flat, even lighting; NO shadows to keep HSV detection stable
    w.append("DirectionalLight { direction 0.3 -0.4 -1 intensity 1.1 "
             "ambientIntensity 0.7 castShadows FALSE }")
    w.append("")

    # floor (top face at z = 0)
    fw = (config.ARENA_X_MAX - config.ARENA_X_MIN) + 2 * config.WALL_THICKNESS
    fh = (config.ARENA_Y_MAX - config.ARENA_Y_MIN) + 2 * config.WALL_THICKNESS
    w.append(solid("FLOOR", f"0 0 -0.025", f"{fw} {fh} 0.05", COLORS["floor"],
                   bounding=False))
    # walls
    wh = config.WALL_HEIGHT
    t = config.WALL_THICKNESS
    cx, cy = (config.ARENA_X_MAX + config.ARENA_X_MIN) / 2, (config.ARENA_Y_MAX + config.ARENA_Y_MIN) / 2
    w.append(solid("WALL_N", f"{cx} {config.ARENA_Y_MAX + t / 2} {wh / 2}",
                   f"{fw} {t} {wh}", COLORS["wall"]))
    w.append(solid("WALL_S", f"{cx} {config.ARENA_Y_MIN - t / 2} {wh / 2}",
                   f"{fw} {t} {wh}", COLORS["wall"]))
    w.append(solid("WALL_E", f"{config.ARENA_X_MAX + t / 2} {cy} {wh / 2}",
                   f"{t} {fh} {wh}", COLORS["wall"]))
    w.append(solid("WALL_W", f"{config.ARENA_X_MIN - t / 2} {cy} {wh / 2}",
                   f"{t} {fh} {wh}", COLORS["wall"]))

    # shelves + crates
    for x, y, sx, sy, h, d in config.SHELVES:
        w.append(solid(d, f"{x} {y} {h / 2}", f"{sx} {sy} {h}", COLORS["shelf"]))
    for x, y, sx, sy, h, d in config.CRATES:
        w.append(solid(d, f"{x} {y} {h / 2}", f"{sx} {sy} {h}", COLORS["crate"]))

    # floor patches — NO boundingObject (non-colliding)
    px, py, psx, psy = config.PICKUP_AREA
    w.append(solid("PICKUP_AREA", f"{px} {py} 0.005", f"{psx} {psy} 0.01",
                   COLORS["pickup"], bounding=False))
    zx, zy, zsx, zsy = config.ZONE_A
    w.append(solid("ZONE_A", f"{zx} {zy} 0.005", f"{zsx} {zsy} 0.01",
                   COLORS["zone_a"], bounding=False))
    zx, zy, zsx, zsy = config.ZONE_B
    w.append(solid("ZONE_B", f"{zx} {zy} 0.005", f"{zsx} {zsy} 0.01",
                   COLORS["zone_b"], bounding=False))

    # packages — pushable Solids with physics
    for defname, x, y, color in config.PACKAGES:
        s = config.PKG_SIZE
        phys = "Physics { density -1 mass 0.5 }"
        w.append(solid(defname, f"{x} {y} {s / 2}", f"{s} {s} {s}",
                       COLORS[color], physics=phys))

    # final-eval hook: dynamic obstacle, parked far away & non-colliding for now
    w.append(solid(config.DYN_OBSTACLE_DEF, "0 -2.85 0.2", "0.4 0.4 0.4",
                   COLORS["crate"], physics="Physics { density -1 mass 0.5 }"))

    w.append(build_robot())
    w.append("")

    out_path = os.path.normpath(OUT)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        f.write("\n".join(w))
    print(f"wrote {out_path} ({os.path.getsize(out_path)} bytes)")


if __name__ == "__main__":
    main()
