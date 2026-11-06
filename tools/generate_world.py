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


def build_robot(controller, def_name="INTELLIBOT", robot_name="INTELLIBOT",
                start_pos=None, start_rot=None, controller_args=None,
                body_color=None, has_arm=False):
    """Generates differential-drive robot with optional 2R mobile manipulator arm."""
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

    cam_vis = (f"Pose {{ translation -0.01 0 0 children [ Shape {{ appearance "
               f"{mat(COLORS['sensor'])} geometry Box {{ size 0.02 0.04 0.03 }} }} ] }}")

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
        f"""    Camera {{
      translation {config.CAMERA_MOUNT_X} 0 {config.CAMERA_MOUNT_Z}
      children [ {cam_vis} ]
      name "{config.CAMERA_NAME}"
      fieldOfView {config.CAMERA_FOV}
      width {config.CAMERA_WIDTH}
      height {config.CAMERA_HEIGHT}
      recognition Recognition {{ frameColor 0.93 0.83 0 }}
    }}""",
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

    # Mobile manipulator 2R arm (Phase 14)
    if has_arm:
        arm_vrml = f"""    LinearJoint {{
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
                            LinearJoint {{
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
                            LinearJoint {{
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
  boundingObject Pose {{ translation 0 0 {body_z} children [
    Box {{ size {bl} {bw} {bh} }} ] }}
  physics Physics {{ density -1 mass {body_mass} }}
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
                     COLORS[color], physics="Physics { density -1 mass 0.5 }")
        body = body.replace(
            f'name "{defname.lower()}"',
            'model "%s"\n  recognitionColors [ %s ]\n  name "%s"'
            % (defname, pkg_rgb[color], defname.lower()))
        w.append(body)

    # final-eval hook: dynamic obstacle, parked in a free corner off every route
    dx, dy = config.DYN_OBSTACLE_PARK
    w.append(solid(config.DYN_OBSTACLE_DEF, f"{dx} {dy} 0.2", "0.4 0.4 0.4",
                   COLORS["crate"], physics="Physics { density -1 mass 5 }"))

    # Moving obstacles (Supervisor-driven)
    w.append(solid("CART_1", "-2.5 2.4 0.15", "0.45 0.35 0.3",
                   COLORS["cart_1"], physics="Physics { density -1 mass 4.0 }"))
    w.append(solid("CART_2", "0.8 -2.4 0.15", "0.4 0.4 0.3",
                   COLORS["cart_2"], physics="Physics { density -1 mass 4.0 }"))

    w.append(build_robot(controller))
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
        '  coordinateSystem "NUE"',
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
