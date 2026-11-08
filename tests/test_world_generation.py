"""World generator: determinism, validity, config consistency (no Webots)."""

import os
import re

import pytest

import config
import generate_world
import shelving
from arm_kinematics import PlanarPitchArm
from validate_world import validate_text

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
TEXT = generate_world.build_world("intellibot_controller")


def test_generator_is_deterministic():
    assert generate_world.build_world("intellibot_controller") == TEXT
    assert generate_world.build_world_v2() == generate_world.build_world_v2()


def test_generated_world_passes_static_validation():
    assert validate_text(TEXT) == []


@pytest.mark.parametrize("name", [n for n in generate_world.WORLDS if "v2" not in n])
def test_worlds_on_disk_are_up_to_date(name):
    with open(os.path.join(ROOT, "worlds", name)) as fh:
        on_disk = fh.read()
    assert on_disk == generate_world.build_world(generate_world.WORLDS[name]), \
        "run: python tools/generate_world.py"
    with open(os.path.join(ROOT, "worlds_output", name)) as fh:
        assert fh.read() == on_disk


def test_validator_catches_the_bugs_that_were_fixed():
    bad_fov = TEXT.replace("fieldOfView 1\n", "fieldOfView 3.14159265\n", 1)
    assert any("fieldOfView" in e for e in validate_text(bad_fov))
    assert any("unknown Webots node type: LinearJoint" in e
               for e in validate_text(TEXT.replace("SliderJoint {", "LinearJoint {", 1)))
    assert any("ENU" in e for e in validate_text(TEXT.replace('"ENU"', '"NUE"')))
    no_rack = TEXT.replace("DEF DEST_RACK_B ", "")
    assert any("DEST_RACK_B" in e for e in validate_text(no_rack))
    no_latch = TEXT.replace(f'name "{config.GRIPPER_CONNECTOR_NAME}"', 'name "x"')
    assert any(config.GRIPPER_CONNECTOR_NAME in e for e in validate_text(no_latch))


def test_v2_world_uses_valid_nodes_and_enu():
    v2 = generate_world.build_world_v2()
    assert "LinearJoint" not in v2 and 'coordinateSystem "ENU"' in v2
    assert not [e for e in validate_text(v2, expect_arm=False)
                if "unknown Webots node" in e or "unbalanced" in e or "ENU" in e]


def test_arm_starts_in_the_stow_pose():
    q = PlanarPitchArm.from_config().inverse(*config.ARM_STOW_TCP, 0.0)
    positions = [float(v) for v in re.findall(
        r"HingeJointParameters \{ axis 0 -1 0 anchor [^}]*position ([-\d.e]+)", TEXT)]
    assert positions == pytest.approx(list(q), abs=1e-5)
    rotations = [float(v) for v in re.findall(r"rotation 0 -1 0 ([-\d.e]+)", TEXT)]
    assert rotations == pytest.approx(list(q), abs=1e-5)       # endPoint matches position
    for name, (lo, hi) in zip(config.ARM_MOTOR_NAMES, config.ARM_JOINT_LIMITS_RAD):
        m = re.search(rf'name "{name}"[^}}]*minPosition ([-\d.e]+) maxPosition ([-\d.e]+)', TEXT)
        assert (float(m.group(1)), float(m.group(2))) == pytest.approx((lo, hi))


def test_every_parcel_has_four_passive_latches_and_config_mass():
    for def_name, *_ in config.PACKAGES:
        start = TEXT.index(f"DEF {def_name} Solid")
        block = TEXT[start:TEXT.index("DEF ", start + 5)]
        assert block.count('type "passive"') == 4
        assert block.count(f'model "{config.CONNECTOR_MODEL}"') == 4
        assert "snap FALSE" in block
        assert f"mass {config.PKG_MASS:g}" in block
    assert TEXT.count('type "active"') == 1


def test_racks_are_where_the_config_says():
    for rack in shelving.build_racks().values():
        m = re.search(rf"DEF {rack.def_name} Solid \{{\s*translation ([^\n]+)", TEXT)
        x, y, z = map(float, m.group(1).split())
        assert (x, y, z) == pytest.approx((rack.center[0], rack.center[1], 0.0))


def test_robot_carries_counterweight_and_post_collision():
    robot = TEXT[TEXT.index("DEF INTELLIBOT Robot"):]
    m = re.search(r"\n  physics Physics \{ density -1 mass ([\d.]+) centerOfMass \[ ([-\d.e]+)", robot)
    assert float(m.group(1)) == pytest.approx(config.BODY_MASS + config.ARM_COUNTERWEIGHT_MASS)
    assert float(m.group(2)) < 0                                    # COM shifted to the rear
    assert re.search(r"\n  boundingObject Group \{ children \[ Pose \{ translation 0 0", robot)
