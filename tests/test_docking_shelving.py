"""Docking (LiDAR face fit + fine positioning) and destination-rack geometry."""

import math

import numpy as np
import pytest

import config
import shelving
from a_star import a_star
from docking import Docker, face_range, refine_parcel_from_scan
from offline_mission_sim import KinematicWorld
from path_utils import plan_to
from warehouse_grid import build_warehouse_grid

P3 = (-3.1, -0.5)


def world_at(x, y, th):
    w = KinematicWorld(start=(x, y, th))
    return w


# ------------------------------------------------------------- LiDAR fits
def test_face_range_is_median_of_front_window():
    angles = np.radians(np.arange(-10, 11))
    scan = np.full(angles.shape, 0.5)
    scan[10] = 0.1                                   # one outlier straight ahead
    scan[0] = np.inf
    assert face_range(scan, angles) == pytest.approx(0.5)
    assert face_range(np.full(angles.shape, np.inf), angles) is None


def test_refine_frontal_view_recovers_centre_and_heading():
    w = world_at(-2.5, -0.5, math.pi)
    scan, ang = w.scan()
    line = refine_parcel_from_scan(tuple(w.pose), scan, ang, (-3.02, -0.44),
                                   preferred_heading=math.pi)
    assert line is not None
    assert line.face_point == pytest.approx((-3.0, -0.5), abs=0.01)
    assert abs(math.remainder(line.heading - math.pi, 2 * math.pi)) < math.radians(1)


def test_refine_oblique_corner_view_picks_the_aisle_face():
    """Seen from the south-east the LiDAR sees two faces (a corner): the
    centre comes from the corner and the east (aisle) face is chosen."""
    w = world_at(-2.6, -1.0, math.atan2(0.5, -0.5))
    scan, ang = w.scan()
    line = refine_parcel_from_scan(tuple(w.pose), scan, ang, (-3.05, -0.55),
                                   preferred_heading=math.pi)
    assert line is not None
    cx = line.face_point[0] + math.cos(line.heading) * config.PKG_SIZE / 2
    cy = line.face_point[1] + math.sin(line.heading) * config.PKG_SIZE / 2
    assert (cx, cy) == pytest.approx(P3, abs=0.02)
    assert abs(math.remainder(line.heading - math.pi, 2 * math.pi)) < math.radians(3)


def test_refine_rejects_estimates_far_from_any_parcel():
    w = world_at(0.0, -0.3, 0.0)
    scan, ang = w.scan()
    assert refine_parcel_from_scan(tuple(w.pose), scan, ang, (2.0, -2.5)) is None


# ----------------------------------------------------------------- docker
def run_docker(w, docker, max_t=60.0, dt=0.032):
    t = 0.0
    while docker.status == "RUNNING" and t < max_t:
        scan, ang = w.scan()
        v, om = docker.update(tuple(w.pose), scan, ang, t)
        w.step_base(v, om, dt)
        t += dt
    return docker


@pytest.mark.parametrize("start", [(-2.55, -0.5, math.pi), (-2.45, -0.36, 2.6),
                                   (-2.7, -0.70, -2.8)])
def test_docker_reaches_parcel_standoff_within_tolerance(start):
    w = world_at(*start)
    d = run_docker(w, Docker(None, parcel_estimate=(-3.03, -0.45),
                             preferred_heading=math.pi, log=lambda m: None))
    assert d.status == "DONE", d.reason
    assert abs(d.measured_range - config.ARM_DOCK_PICK_STANDOFF) <= 2 * config.ARM_DOCK_RANGE_TOL
    assert abs(d.final_lateral_error) <= config.ARM_DOCK_LATERAL_TOL
    assert abs(d.final_heading_error) <= 2 * config.ARM_DOCK_HEADING_TOL
    # true geometry: robot centre on the parcel's centre line
    assert abs(w.pose[1] - P3[1]) < 0.015


def test_docker_reaches_rack_standoff():
    slot = shelving.slots_for_zone("B")[0]
    line = shelving.dock_line_for_slot(slot)
    gx, gy, _ = shelving.predock_pose(line)
    w = world_at(gx - 0.25, gy + 0.12, 0.4)
    d = run_docker(w, Docker(line, log=lambda m: None))
    assert d.status == "DONE", d.reason
    assert d.range_source == "lidar"
    assert w.pose[0] + d.measured_range == pytest.approx(config.DEST_RACK_FRONT_X, abs=0.01)


def test_docker_aborts_when_too_close_and_on_invalid_pose():
    slot = shelving.slots_for_zone("B")[0]
    line = shelving.dock_line_for_slot(slot)
    d = Docker(line, log=lambda m: None)
    d.phase = "CREEP"
    w = world_at(config.DEST_RACK_FRONT_X - 0.15, slot.center[1], 0.0)
    scan, ang = w.scan()
    d.update(tuple(w.pose), scan, ang, 0.0)
    assert d.status == "FAILED" and "TOO_CLOSE" in d.reason
    d2 = Docker(line, log=lambda m: None)
    d2.update((math.nan, 0.0, 0.0), None, None, 0.0)
    assert d2.status == "FAILED" and d2.reason == "INVALID_POSE"


def test_docker_times_out():
    d = Docker(shelving.dock_line_for_slot(shelving.all_slots()[0]), log=lambda m: None)
    d.update((0.0, 0.0, 0.0), None, None, 0.0)
    d.update((0.0, 0.0, 0.0), None, None, config.ARM_DOCK_TIMEOUT + 1.0)
    assert d.status == "FAILED" and "TIMEOUT" in d.reason


# --------------------------------------------------------------- shelving
def test_slot_ids_levels_and_reachability():
    slots = shelving.all_slots()
    n = len(config.DEST_RACKS) * config.DEST_RACK_BAYS * len(config.DEST_RACK_LEVEL_TOPS)
    assert len(slots) == n and len({s.slot_id for s in slots}) == n
    for s in slots:
        assert s.reachable == (s.level in config.DEST_RACK_REACHABLE_LEVELS)
        assert s.center[2] == pytest.approx(s.surface_z + config.PKG_SIZE / 2)
    assert [s.slot_id for s in shelving.slots_for_zone("B")] == \
        ["RB-L1-B1", "RB-L1-B2", "RB-L2-B1", "RB-L2-B2"]


def test_slots_fit_inside_rack_and_between_levels():
    for s in shelving.all_slots():
        x, y, z = s.center
        h = config.PKG_SIZE / 2
        assert config.DEST_RACK_FRONT_X < x - h
        assert x + h < config.DEST_RACK_FRONT_X + config.DEST_RACK_DEPTH - config.DEST_RACK_PANEL_THICKNESS
        rack_y = config.DEST_RACKS[s.zone][0]
        bay_lo = rack_y - config.DEST_RACK_WIDTH / 2 + (s.bay - 1) * config.DEST_RACK_BAY_WIDTH
        # open gripper (fingers + pads) fits between the uprights of the bay
        half_gripper = config.GRIPPER_PALM_SIZE[1] / 2
        assert bay_lo + config.DEST_RACK_POST_SIZE < y - half_gripper
        assert y + half_gripper < bay_lo + config.DEST_RACK_BAY_WIDTH
        tops = config.DEST_RACK_LEVEL_TOPS
        if s.level < len(tops):
            clearance = (tops[s.level] - config.DEST_RACK_PLATFORM_THICKNESS) - (z + h)
            assert clearance > config.ARM_PLACE_HOVER + 0.02


def test_racks_are_obstacles_in_the_planning_grid_and_inside_the_arena():
    g = build_warehouse_grid()
    for cx, cy, sx, sy in config.DEST_RACK_FOOTPRINTS:
        assert config.ARENA_X_MIN < cx - sx / 2 and cx + sx / 2 < config.ARENA_X_MAX
        r, c = g.world_to_cell(cx, cy)
        assert g.static[r, c]
    # the zone patches themselves stay drivable
    occ = g.inflate(config.INFLATION_RADIUS)
    for zx, zy in config.ZONE_OF.values():
        assert not occ[g.world_to_cell(zx, zy)]


def test_predock_poses_are_free_and_reachable_from_the_pickup_area():
    g = build_warehouse_grid()
    occ = g.inflate(config.INFLATION_RADIUS)
    for slot in shelving.all_slots():
        if not slot.reachable:
            continue
        x, y, _ = shelving.predock_pose(shelving.dock_line_for_slot(slot))
        assert not occ[g.world_to_cell(x, y)], slot.slot_id
        wps, info = plan_to(g, (-2.55, 0.0), (x, y), resample_spacing=0.3,
                            inflation_radius=config.INFLATION_RADIUS)
        assert wps is not None and not info.get("snapped"), slot.slot_id


def test_racks_keep_two_distinct_routes_to_each_zone():
    """Adding the racks must not close the outer corridor (alternate route)."""
    g = build_warehouse_grid()
    for zone in config.DEST_RACKS:
        x, y, _ = shelving.predock_pose(shelving.dock_line_for_slot(
            shelving.slots_for_zone(zone)[0]))
        for yy in np.arange(-2.0, 2.01, 0.2):
            g.add_obstacle_world(0.0, float(yy), 0.3)
        wps, _ = plan_to(g, (-2.55, 0.0), (x, y), resample_spacing=0.3,
                         inflation_radius=config.INFLATION_RADIUS)
        assert wps is not None, zone
        g.clear_dynamic()


def test_place_targets_follow_measured_range():
    slot = shelving.slots_for_zone("A")[0]
    nominal = shelving.place_tcp_for_slot(slot)
    nearer = shelving.place_tcp_for_slot(slot, measured_range=config.ARM_DOCK_PLACE_STANDOFF - 0.01)
    assert nearer["x"] == pytest.approx(nominal["x"] - 0.01)
    assert nominal["z_hover"] > nominal["z_release"] > slot.surface_z + config.PKG_SIZE / 2


def test_audit_and_occupancy():
    slot = shelving.slots_for_zone("B")[0]
    ok, err = shelving.audit_placement(slot.center, slot)
    assert ok and err["xy"] == 0
    ok, err = shelving.audit_placement((slot.center[0] + 0.1, slot.center[1], 0.1), slot)
    assert not ok
    occ = shelving.RackOccupancy()
    assert occ.next_free("B").slot_id == "RB-L1-B1"
    occ.mark("RB-L1-B1", "P3")
    occ.mark("RB-L1-B2", "blocked")
    assert occ.next_free("B").slot_id == "RB-L2-B1"
    for s in shelving.slots_for_zone("B"):
        occ.mark(s.slot_id, "x")
    assert occ.next_free("B") is None


def test_dock_line_geometry():
    line = shelving.dock_line_for_parcel(P3, math.pi)
    assert line.face_point == pytest.approx((-3.0, -0.5))
    x, y, th = line.docked_pose()
    assert (x, y) == pytest.approx((-3.0 + config.ARM_DOCK_PICK_STANDOFF, -0.5))
    assert line.along_distance(x, y) == pytest.approx(config.ARM_DOCK_PICK_STANDOFF)
    assert line.lateral_error(x, y + 0.02) == pytest.approx(-0.02)
