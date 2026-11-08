"""validate_world.py — static checks of the generated .wbt worlds (no Webots).

What it checks (each is a cheap proxy for a Webots load error/warning):
    * header "#VRML_SIM R2025a utf8" and coordinateSystem "ENU"
    * balanced { } and [ ]
    * every node type is a real Webots base node (catches e.g. "LinearJoint",
      which does not exist — the slider joint node is "SliderJoint")
    * DEF names are unique
    * the robot contains every device name the controllers look up
      (config.py), each exactly once
    * planar cameras have 0 < fieldOfView < pi
    * required DEFs exist (parcels, shelves, crates, destination racks, robot)
    * the generator is deterministic (two runs -> identical text)

Run:  python tools/validate_world.py            (validates worlds/warehouse*.wbt)
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "controllers", "intellibot_controller"))
sys.path.insert(0, HERE)
import config  # noqa: E402

# Webots R2025a base nodes (from the Webots reference manual node list)
WEBOTS_NODES = set("""
Accelerometer Altimeter Appearance Background BallJoint BallJointParameters
Billboard Box Brake CadShape Camera Capsule Charger Color Compass Cone Connector
ContactProperties Coordinate Cylinder Damping Display DistanceSensor
DirectionalLight ElevationGrid Emitter Fluid Focus Fog GPS Group Gyro
Hinge2Joint HingeJoint HingeJointParameters ImageTexture ImmersionProperties
IndexedFaceSet IndexedLineSet InertialUnit JointParameters LED Lens LensFlare
Lidar LightSensor LinearMotor Material Mesh Muscle Normal PBRAppearance Pen
Physics Plane PointLight PointSet Pose PositionSensor Propeller Radar
RangeFinder Receiver Recognition Robot RotationalMotor Shape Skin SliderJoint
Slot Solid SolidReference Speaker Sphere SpotLight TextureCoordinate
TextureTransform TouchSensor Track TrackWheel Transform Viewpoint WorldInfo
Zoom VacuumGripper
""".split())


def mission_device_names():
    """Every device name intellibot_controller / manip_check will request."""
    names = [config.LIDAR_NAME, config.CAMERA_NAME, config.REAR_CAMERA_NAME,
             config.GPS_NAME, config.IMU_NAME, config.LEFT_MOTOR, config.RIGHT_MOTOR,
             config.LEFT_ENCODER, config.RIGHT_ENCODER, config.LED_NAME,
             config.PEN_NAME, config.DISPLAY_NAME, "bumper_f", "bumper_r",
             config.GRIPPER_PALM_RANGE_NAME, config.GRIPPER_CONNECTOR_NAME]
    names += list(config.ARM_MOTOR_NAMES) + list(config.ARM_SENSOR_NAMES)
    names += list(config.GRIPPER_FINGER_MOTOR_NAMES)
    names += list(config.GRIPPER_FINGER_SENSOR_NAMES)
    names += list(config.GRIPPER_FINGER_TOUCH_NAMES)
    return names


def required_defs():
    import shelving
    defs = [d for (d, _x, _y, _c) in config.PACKAGES]
    defs += [d for (*_r, d) in config.SHELVES] + [d for (*_r, d) in config.CRATES]
    defs += [r.def_name for r in shelving.build_racks().values()]
    defs += ["INTELLIBOT", "VIEWPOINT", "FLOOR"]
    return defs


def _robot_block(text, def_name="INTELLIBOT"):
    start = text.find(f"DEF {def_name} Robot {{")
    if start < 0:
        return ""
    depth = 0
    for i in range(text.index("{", start), len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
    return text[start:]


def validate_text(text, expect_arm=True):
    """Return a list of error strings (empty = all checks passed)."""
    errors = []
    if not text.startswith("#VRML_SIM R2025a utf8"):
        errors.append("missing/incorrect #VRML_SIM header")
    if 'coordinateSystem "ENU"' not in text:
        errors.append('WorldInfo.coordinateSystem is not "ENU"')
    body = re.sub(r'"[^"]*"', '""', text)          # ignore braces inside strings
    body = "\n".join(line.split("#", 1)[0] for line in body.splitlines())
    if body.count("{") != body.count("}"):
        errors.append(f"unbalanced braces: {body.count('{')} vs {body.count('}')}")
    if body.count("[") != body.count("]"):
        errors.append(f"unbalanced brackets: {body.count('[')} vs {body.count(']')}")
    for node in sorted(set(re.findall(r"\b([A-Z][A-Za-z0-9]*) \{", body))):
        if node not in WEBOTS_NODES:
            errors.append(f"unknown Webots node type: {node}")
    defs = re.findall(r"\bDEF (\w+) ", body)
    dup = sorted({d for d in defs if defs.count(d) > 1})
    if dup:
        errors.append(f"duplicate DEF names: {dup}")
    for d in required_defs():
        if d not in defs:
            errors.append(f"required DEF missing: {d}")
    robot = _robot_block(text)
    names = re.findall(r'\bname "([^"]+)"', robot)
    wanted = mission_device_names() if expect_arm else mission_device_names()[:14]
    for n in wanted:
        c = names.count(n)
        if c != 1:
            errors.append(f"robot device '{n}' found {c} times (expected 1)")
    for m in re.finditer(r'Camera \{(.*?)\n    \}', robot, re.S):
        block = m.group(1)
        fov = re.search(r"fieldOfView ([0-9.eE+-]+)", block)
        proj = re.search(r'projection "(\w+)"', block)
        if fov and (proj is None or proj.group(1) == "planar"):
            v = float(fov.group(1))
            if not (0.0 < v < 3.14159):
                errors.append(f"planar camera fieldOfView {v} not in (0, pi)")
    return errors


def main():
    import generate_world
    worlds = [n for n in generate_world.WORLDS if "v2" not in n]
    ok = True
    # determinism: build twice, compare
    for name, ctrl in generate_world.WORLDS.items():
        build = generate_world.build_world_v2 if "v2" in name else generate_world.build_world
        if build(ctrl) != build(ctrl):
            print(f"[FAIL] {name}: generator is not deterministic")
            ok = False
    for name in worlds:
        path = os.path.join(HERE, "..", "worlds", name)
        with open(path) as fh:
            text = fh.read()
        expected = generate_world.build_world(generate_world.WORLDS[name])
        errs = validate_text(text)
        if text != expected:
            errs.append("file on disk differs from the generator output "
                        "(run python tools/generate_world.py)")
        if errs:
            ok = False
            print(f"[FAIL] {name}")
            for e in errs:
                print("   -", e)
        else:
            print(f"[ OK ] {name}: header, syntax, node types, DEFs, "
                  f"{len(mission_device_names())} devices, camera FOV")
    v2 = os.path.join(HERE, "..", "worlds", "warehouse_v2.wbt")
    with open(v2) as fh:
        v2_text = fh.read()
    v2_err = [e for e in validate_text(v2_text, expect_arm=False)
              if "required DEF" not in e and "robot device" not in e]
    print(("[ OK ]" if not v2_err else "[WARN]") + " warehouse_v2.wbt (syntax/node types only):"
          + ("" if not v2_err else " " + "; ".join(v2_err)))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
