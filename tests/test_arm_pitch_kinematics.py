"""Mission-robot arm: FK/IK, joint limits, workspace and clearance.

These tests are the offline proof that the arm built into the world can
reach every place it is asked to go WITHOUT touching the robot (body,
bumpers, cameras, post), the carried parcel or the rack.
"""

import math

import pytest

import config
import shelving
from arm_kinematics import (PlanarPitchArm, arm_clearance, robot_obstacle_rects,
                            segment_rect_distance)

ARM = PlanarPitchArm.from_config()
FL = config.GRIPPER_FINGER_SIZE[0]
W2P = config.GRIPPER_WRIST_TO_PALM
P = config.PKG_SIZE


def clearance(q, holding=False, extra=None, target=None):
    obs = robot_obstacle_rects()
    if extra:
        obs.update(extra)
    return arm_clearance(ARM, q, wrist_to_palm=W2P, finger_len=FL,
                         holding_size=P if holding else None, obstacles=obs,
                         target_parcel=target)


def line(a, b, step=0.005):
    n = max(2, int(math.dist(a, b) / step))
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
            for i in range(n + 1)]


# ----------------------------------------------------------- kinematics
def test_tcp_offset_matches_gripper_geometry():
    assert config.ARM_TCP_OFFSET == pytest.approx(
        config.GRIPPER_WRIST_TO_PALM + config.GRIPPER_PALM_GAP + P / 2)


@pytest.mark.parametrize("target", [(0.40, 0.10), (0.085, 0.28), (0.38, 0.265),
                                    (0.38, 0.525), (0.13, 0.265), (0.30, 0.40)])
def test_ik_fk_round_trip(target):
    q = ARM.inverse(*target, 0.0)
    assert q is not None
    fk = ARM.forward(q)
    assert fk["TCP"][0] == pytest.approx(target[0], abs=1e-9)
    assert fk["TCP"][1] == pytest.approx(target[1], abs=1e-9)
    assert fk["phi"] == pytest.approx(0.0, abs=1e-9)        # gripper horizontal


def test_elbow_up_and_down_are_distinct_solutions():
    up = ARM.inverse(0.38, 0.265, 0.0, elbow_up=True, check_limits=False)
    down = ARM.inverse(0.38, 0.265, 0.0, elbow_up=False, check_limits=False)
    assert up[1] < 0 < down[1]
    assert ARM.forward(up)["E"][1] > ARM.forward(down)["E"][1]   # "up" is higher
    for q in (up, down):
        assert ARM.forward(q)["TCP"] == pytest.approx((0.38, 0.265), abs=1e-9)


def test_out_of_reach_and_joint_limit_reasons():
    q, reason = ARM.solve(1.5, 0.2)
    assert q is None and reason == "OUT_OF_REACH"
    q, reason = ARM.solve(float("nan"), 0.2)
    assert q is None and reason == "INVALID_TARGET"
    # elbow DOWN for the stow pose needs the wrist far beyond its limit
    q, reason = ARM.solve(*config.ARM_STOW_TCP, 0.0, elbow_up=False)
    assert q is None and reason.startswith("JOINT_LIMIT")


def test_clamp_keeps_commands_inside_limits():
    q = ARM.clamp((10.0, -10.0, 0.0))
    assert q[0] == config.ARM_JOINT_LIMITS_RAD[0][1]
    assert q[1] == config.ARM_JOINT_LIMITS_RAD[1][0]


def test_reach_ratio_and_singularity_margin():
    """Every working target keeps >= 15 % margin to full stretch."""
    targets = [shelving.pick_tcp_for_parcel()]
    for slot in shelving.all_slots():
        if slot.reachable:
            pl = shelving.place_tcp_for_slot(slot)
            targets.append({"x": pl["x"], "z": pl["z_hover"]})
    for t in targets:
        assert ARM.reach_ratio(t["x"], t["z"]) < 0.85


# --------------------------------------------------- workspace / shelves
def test_every_reachable_slot_is_reachable_and_l3_is_not():
    for slot in shelving.all_slots():
        pl = shelving.place_tcp_for_slot(slot)
        targets = [(pl["pre_x"], pl["z_hover"]), (pl["x"], pl["z_hover"]),
                   (pl["x"], pl["z_release"]), (pl["retract_x"], pl["z_release"])]
        ok = all(ARM.inverse(x, z, 0.0) is not None for x, z in targets)
        assert ok == slot.reachable, slot.slot_id


def test_floor_parcel_reachable_within_docking_tolerance():
    """The grasp must stay reachable for the whole docking range tolerance."""
    for dr in (-0.02, 0.0, 0.02):
        g = shelving.pick_tcp_for_parcel(config.ARM_DOCK_PICK_STANDOFF + dr)
        for x, z in ((g["x"], g["z"]),
                     (g["x"] - config.ARM_PREGRASP_BACKOFF, g["z"]),
                     (g["x"] - config.ARM_PREGRASP_BACKOFF, g["z"] + config.ARM_PREGRASP_RAISE)):
            assert ARM.inverse(x, z, 0.0) is not None, (dr, x, z)


# --------------------------------------------------------------- clearance
def _pick_path():
    g = shelving.pick_tcp_for_parcel()
    xg, zg = g["x"], g["z"]
    xp = xg - config.ARM_PREGRASP_BACKOFF
    target = (config.ARM_DOCK_PICK_STANDOFF, config.ARM_DOCK_PICK_STANDOFF + P, 0.0, P)
    approach = ([((xp, zg + config.ARM_PREGRASP_RAISE), False)]
                + [(p, False) for p in line((xp, zg + config.ARM_PREGRASP_RAISE), (xp, zg))]
                + [(p, False) for p in line((xp, zg), (xg, zg))])
    carry = ([(p, True) for p in line((xg, zg), (xg, zg + config.ARM_LIFT_CHECK_DZ))]
             + [(p, True) for p in line((xg, zg + config.ARM_LIFT_CHECK_DZ),
                                        (xg, config.ARM_SAFE_TRANSIT_Z))]
             + [(p, True) for p in line((xg, config.ARM_SAFE_TRANSIT_Z), config.ARM_STOW_TCP)])
    return approach, carry, target


def test_pick_sequence_never_touches_robot_or_parcel():
    approach, carry, target = _pick_path()
    for tcp, _h in approach:
        c, who = clearance(ARM.inverse(*tcp, 0.0), target=target)
        assert c > 0.004, (tcp, who, c)
    # (the first carry point is the grasp itself: parcel still on the floor)
    for tcp, _h in carry[1:]:
        c, who = clearance(ARM.inverse(*tcp, 0.0), holding=True)
        assert c > 0.0, (tcp, who, c)
    lifted = [p for p, _h in carry if p[1] >= P / 2 + 0.01]
    assert min(clearance(ARM.inverse(*p, 0.0), holding=True)[0] for p in lifted) > 0.004


@pytest.mark.parametrize("level", [1, 2])
def test_place_sequence_never_touches_robot_rack_or_parcel(level):
    slot = [s for s in shelving.slots_for_zone("B") if s.level == level][0]
    pl = shelving.place_tcp_for_slot(slot)
    rack = shelving.rack_rects_robot_frame()
    holding = (line(config.ARM_STOW_TCP, (pl["pre_x"], pl["z_hover"]))
               + line((pl["pre_x"], pl["z_hover"]), (pl["x"], pl["z_hover"])))
    for tcp in holding:
        c, who = clearance(ARM.inverse(*tcp, 0.0), holding=True, extra=rack)
        assert c > 0.004, (tcp, who, c)
    # lowering: the parcel ends ARM_PLACE_RELEASE_DROP above its own platform
    for tcp in line((pl["x"], pl["z_hover"]), (pl["x"], pl["z_release"])):
        c, who = clearance(ARM.inverse(*tcp, 0.0), holding=True, extra=rack)
        assert c >= config.ARM_PLACE_RELEASE_DROP - 1e-6, (tcp, who, c)
    empty = (line((pl["x"], pl["z_release"]), (pl["retract_x"], pl["z_release"]))
             + line((pl["retract_x"], pl["z_release"]), (pl["pre_x"], pl["z_release"]))
             + line((pl["pre_x"], pl["z_release"]), config.ARM_STOW_TCP))
    placed = (pl["x"] - P / 2, pl["x"] + P / 2, slot.surface_z, slot.surface_z + P)
    rack_and_parcel = dict(rack, placed_parcel=placed)
    for tcp in empty:
        q = ARM.inverse(*tcp, 0.0)
        c, who = arm_clearance(ARM, q, wrist_to_palm=W2P, finger_len=FL,
                               obstacles=dict(robot_obstacle_rects(), **rack),
                               target_parcel=placed)
        assert c > 0.004, (tcp, who, c)


def test_stow_pose_is_compact_and_clear_of_lidar_and_cameras():
    q = ARM.inverse(*config.ARM_STOW_TCP, 0.0)
    fk = ARM.forward(q)
    tcp = fk["TCP"]
    parcel_bottom = tcp[1] - P / 2
    # carried parcel above the LiDAR plane and above both camera housings
    assert parcel_bottom > config.LIDAR_MOUNT_Z + 0.03
    assert parcel_bottom > config.CAMERA_MOUNT_Z + config.CAMERA_HOUSING_SIZE[2]
    # everything stays within the bumper line (+2 cm for the parcel)
    xs = [fk[k][0] for k in ("S", "E", "W", "TCP")] + [tcp[0] + P / 2]
    assert max(xs) <= config.BODY_LENGTH / 2 + 0.02 + 0.02
    assert min(xs) >= -config.BODY_LENGTH / 2 - 0.02
    # nothing of the arm below z = LiDAR plane + 3 cm except the post
    for _p0, p1, name in ARM.link_segments(q, W2P, FL):
        assert p1[1] > config.LIDAR_MOUNT_Z + 0.03, name


def _in_camera_frustum(point_xz, cam_x, looks_forward):
    """Is a robot-frame x-z point inside a camera's vertical field of view?"""
    vfov = 2 * math.atan(math.tan(config.CAMERA_FOV / 2) * config.CAMERA_HEIGHT
                         / config.CAMERA_WIDTH)
    dx = (point_xz[0] - cam_x) * (1 if looks_forward else -1)
    dz = point_xz[1] - config.CAMERA_MOUNT_Z
    return dx > config.CAMERA_NEAR and abs(math.atan2(dz, dx)) < vfov / 2


def test_stowed_arm_and_parcel_do_not_occlude_either_camera():
    q = ARM.inverse(*config.ARM_STOW_TCP, 0.0)
    pts = []
    for p0, p1, _n in ARM.link_segments(q, W2P, FL):
        pts += line(p0, p1, 0.01)
    tcp = ARM.forward(q)["TCP"]
    pts += [(tcp[0] + dx, tcp[1] + dz) for dx in (-P / 2, P / 2) for dz in (-P / 2, P / 2)]
    for p in pts:
        assert not _in_camera_frustum(p, config.CAMERA_MOUNT_X, True), p
        assert not _in_camera_frustum(p, config.REAR_CAMERA_MOUNT_X, False), p


def test_lidar_self_filter_covers_only_the_post():
    """Post distance from the LiDAR < filter radius < bumper distance."""
    post_near = abs(config.ARM_BASE_X) - config.ARM_POST_SIZE / 2
    assert post_near < config.LIDAR_SELF_FILTER_RANGE
    assert config.LIDAR_SELF_FILTER_RANGE < config.BODY_LENGTH / 2 + 0.02


def test_clearance_model_detects_a_collision():
    """Sanity: an arm pose through the body must report negative clearance."""
    q = ARM.inverse(0.10, 0.08, 0.0, check_limits=False)
    c, who = clearance(q)
    assert c < 0, who
    assert segment_rect_distance((0, 0), (1, 0), (0.4, 0.6, -0.1, 0.1)) == 0.0
    assert segment_rect_distance((0, 1), (1, 1), (0.4, 0.6, -0.1, 0.1)) == pytest.approx(0.9)


def test_static_tipping_margin_with_parcel_at_full_reach():
    """COM stays well inside the support polygon (front caster) when lifting."""
    q = ARM.inverse(*(lambda g: (g["x"], g["z"] + config.ARM_LIFT_CHECK_DZ))(
        shelving.pick_tcp_for_parcel()), 0.0)
    fk = ARM.forward(q)
    m_up, m_fore, m_grip = config.ARM_LINK_MASSES
    parts = [
        (config.BODY_MASS + 0.24, 0.0),                       # body + wheels/casters
        (config.ARM_COUNTERWEIGHT_MASS, config.ARM_COUNTERWEIGHT_X),
        (m_up, (fk["S"][0] + fk["E"][0]) / 2), (m_fore, (fk["E"][0] + fk["W"][0]) / 2),
        (m_grip + 2 * config.GRIPPER_FINGER_MASS, fk["W"][0] + W2P),
        (config.PKG_MASS, fk["TCP"][0]),
    ]
    com = sum(m * x for m, x in parts) / sum(m for m, _ in parts)
    front_caster = config.BODY_LENGTH / 2 - 0.04
    assert com < front_caster - 0.05
