"""shelving.py — destination racks, slots and docking lines (pure, no Webots).

Everything here is DERIVED from config.py so that the world generator
(tools/generate_world.py), the planner's occupancy grid (config.STATIC_OBSTACLES
already contains config.DEST_RACK_FOOTPRINTS), the FSM and the tests all use
the same geometry.

World frame ENU (x east, y north, z up); robot frame FLU. Each rack stands
against the east wall and its open front faces WEST (-x). A robot that serves
a rack therefore docks facing EAST (heading 0).

    slot id  : "<prefix>-L<level>-B<bay>", e.g. "RB-L1-B1"
    slot.center : world position of the parcel centre once placed
    DockLine : the straight line the base must sit on to serve a target face

Placement audit
---------------
``audit_placement`` compares a parcel's (ground-truth) position with the slot
it was placed in. The controller uses it ONLY for the evaluation report — the
FSM decides from the robot's own sensors (see pick_place.py).
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import config


# ------------------------------------------------------------------ dataclasses
@dataclass(frozen=True)
class Slot:
    """One placement location on a destination rack."""
    slot_id: str
    zone: str
    rack_def: str
    level: int                       # 1 = lowest platform
    bay: int                         # 1 = southern bay (smaller y)
    center: Tuple[float, float, float]   # parcel centre when placed (world)
    surface_z: float                 # platform top height
    reachable: bool                  # within the arm's workspace


@dataclass(frozen=True)
class DockLine:
    """Line the robot base must sit on to face a target surface.

    face_point : point on the target face (world, z ignored)
    heading    : robot heading when facing the target (rad, world)
    standoff   : desired LiDAR-centre -> face distance when docked (m)
    """
    face_point: Tuple[float, float]
    heading: float
    standoff: float

    @property
    def direction(self) -> Tuple[float, float]:
        """Unit vector from the robot towards the face."""
        return math.cos(self.heading), math.sin(self.heading)

    def pose_at(self, dist_from_face: float) -> Tuple[float, float, float]:
        """Robot pose on the line ``dist_from_face`` metres in front of the face."""
        ux, uy = self.direction
        fx, fy = self.face_point
        return fx - ux * dist_from_face, fy - uy * dist_from_face, self.heading

    def docked_pose(self) -> Tuple[float, float, float]:
        return self.pose_at(self.standoff)

    def along_distance(self, x: float, y: float) -> float:
        """Distance from (x, y) to the face, measured along the line."""
        ux, uy = self.direction
        fx, fy = self.face_point
        return (fx - x) * ux + (fy - y) * uy

    def lateral_error(self, x: float, y: float) -> float:
        """Signed offset of (x, y) from the line (+ = left of the line)."""
        ux, uy = self.direction
        fx, fy = self.face_point
        dx, dy = x - fx, y - fy
        return -dx * uy + dy * ux


@dataclass
class Rack:
    zone: str
    def_name: str
    prefix: str
    center: Tuple[float, float]
    front_x: float
    slots: List[Slot] = field(default_factory=list)


# ------------------------------------------------------------------ geometry
def _bay_center_y(rack_y: float, bay: int) -> float:
    """Bay 1 is the southern (smaller y) bay."""
    n = config.DEST_RACK_BAYS
    w = config.DEST_RACK_BAY_WIDTH
    return rack_y + (bay - (n + 1) / 2.0) * w


def build_racks() -> Dict[str, Rack]:
    """All destination racks with their slots, keyed by zone letter."""
    racks = {}
    half = config.PKG_SIZE / 2.0
    slot_x = config.DEST_RACK_FRONT_X + config.DEST_RACK_SLOT_INSET + half
    for zone, (rack_y, def_name, prefix) in config.DEST_RACKS.items():
        rack = Rack(zone, def_name, prefix,
                    (config.DEST_RACK_CENTER_X, rack_y), config.DEST_RACK_FRONT_X)
        for level, top in enumerate(config.DEST_RACK_LEVEL_TOPS, start=1):
            for bay in range(1, config.DEST_RACK_BAYS + 1):
                rack.slots.append(Slot(
                    slot_id=f"{prefix}-L{level}-B{bay}", zone=zone,
                    rack_def=def_name, level=level, bay=bay,
                    center=(slot_x, _bay_center_y(rack_y, bay), top + half),
                    surface_z=top,
                    reachable=level in config.DEST_RACK_REACHABLE_LEVELS))
        racks[zone] = rack
    return racks


def all_slots() -> List[Slot]:
    return [s for r in build_racks().values() for s in r.slots]


def get_slot(slot_id: str) -> Slot:
    for s in all_slots():
        if s.slot_id == slot_id:
            return s
    raise KeyError(f"unknown slot {slot_id!r}")


def slots_for_zone(zone: str, reachable_only: bool = True) -> List[Slot]:
    """Slots of a zone's rack in fill order: lowest level first, bay 1 first."""
    racks = build_racks()
    if zone not in racks:
        raise KeyError(f"no destination rack for zone {zone!r}")
    slots = [s for s in racks[zone].slots if s.reachable or not reachable_only]
    return sorted(slots, key=lambda s: (s.level, s.bay))


def dock_line_for_slot(slot: Slot) -> DockLine:
    """Racks face west: dock on the slot's y, facing east, at the place standoff."""
    return DockLine(face_point=(config.DEST_RACK_FRONT_X, slot.center[1]),
                    heading=0.0, standoff=config.ARM_DOCK_PLACE_STANDOFF)


def dock_line_for_parcel(parcel_xy: Tuple[float, float],
                         approach_heading: float) -> DockLine:
    """Dock line for a floor parcel approached along ``approach_heading``.

    The face point is the centre of the parcel face that looks back at the
    robot (half a parcel in front of the parcel centre).
    """
    half = config.PKG_SIZE / 2.0
    ux, uy = math.cos(approach_heading), math.sin(approach_heading)
    face = (parcel_xy[0] - ux * half, parcel_xy[1] - uy * half)
    return DockLine(face_point=face, heading=approach_heading,
                    standoff=config.ARM_DOCK_PICK_STANDOFF)


def predock_pose(line: DockLine) -> Tuple[float, float, float]:
    """Navigation goal before docking: on the line, a bit further back."""
    return line.pose_at(line.standoff + config.ARM_DOCK_PREDOCK_EXTRA)


# ------------------------------------------------------- robot-frame helpers
def place_tcp_for_slot(slot: Slot, standoff: Optional[float] = None,
                       measured_range: Optional[float] = None):
    """TCP targets in the ROBOT frame for placing a parcel into ``slot``.

    When docked, the rack front is ``measured_range`` (LiDAR) ahead of the
    robot; the parcel centre must end DEST_RACK_SLOT_INSET + half a parcel
    behind it. Returns dict with x (insert), pre_x (outside the rack),
    z_hover (insertion height), z_release, retract_x.
    """
    if standoff is None:
        standoff = config.ARM_DOCK_PLACE_STANDOFF
    d = standoff if measured_range is None else measured_range
    half = config.PKG_SIZE / 2.0
    x = d + config.DEST_RACK_SLOT_INSET + half
    return {
        "x": x,
        "pre_x": d - config.ARM_PLACE_CLEARANCE - half,
        "z_hover": slot.surface_z + half + config.ARM_PLACE_HOVER,
        "z_release": slot.surface_z + half + config.ARM_PLACE_RELEASE_DROP,
        "retract_x": x - config.ARM_RETRACT_DIST,
    }


def pick_tcp_for_parcel(measured_face_range: Optional[float] = None):
    """TCP grasp target (robot frame) for a floor parcel ``range`` ahead."""
    d = (config.ARM_DOCK_PICK_STANDOFF if measured_face_range is None
         else measured_face_range)
    half = config.PKG_SIZE / 2.0
    return {"x": d + half, "z": half}


def rack_rects_robot_frame(standoff: Optional[float] = None) -> dict:
    """Rack structure in the robot's x-z plane (y = 0) when docked at a bay.

    Used by the arm clearance checks. Posts sit at the bay edges (|y| =
    BAY_WIDTH/2 from the arm plane) and are therefore not in the arm plane.
    """
    if standoff is None:
        standoff = config.ARM_DOCK_PLACE_STANDOFF
    d = standoff
    depth = config.DEST_RACK_DEPTH
    th = config.DEST_RACK_PLATFORM_THICKNESS
    pt = config.DEST_RACK_PANEL_THICKNESS
    tops = config.DEST_RACK_LEVEL_TOPS
    rects = {f"rack_L{i}": (d, d + depth, t - th, t)
             for i, t in enumerate(tops, start=1)}
    rects["rack_kick_plate"] = (d, d + pt, 0.0, tops[0] - th)
    rects["rack_back_panel"] = (d + depth - pt, d + depth, 0.0, tops[-1])
    return rects


# ------------------------------------------------------- verification/audit
def audit_placement(parcel_xyz, slot: Slot, tol_xy: float = 0.04,
                    tol_z: float = 0.02):
    """Ground-truth audit (evaluation only): is the parcel resting in the slot?

    Returns (ok, errors) with errors = {"xy": m, "z": m}.
    """
    px, py, pz = (float(v) for v in parcel_xyz)
    sx, sy, sz = slot.center
    exy = math.hypot(px - sx, py - sy)
    ez = abs(pz - sz)
    return (exy <= tol_xy and ez <= tol_z), {"xy": exy, "z": ez}


class RackOccupancy:
    """The robot's own record of which slots it has filled (per run).

    Kept deliberately simple: the FSM asks for the next free reachable slot
    of the requested zone and marks it after a VERIFIED delivery. A slot
    whose insertion was blocked is marked "blocked" so the next attempt uses
    a different one.
    """

    def __init__(self):
        self.occupied: Dict[str, str] = {}   # slot_id -> parcel id / "blocked"

    def next_free(self, zone: str) -> Optional[Slot]:
        for slot in slots_for_zone(zone, reachable_only=True):
            if slot.slot_id not in self.occupied:
                return slot
        return None

    def mark(self, slot_id: str, what: str):
        self.occupied[slot_id] = what

    def free(self, slot_id: str):
        self.occupied.pop(slot_id, None)

    def slot_of(self, parcel_id: str) -> Optional[str]:
        """Slot this run placed `parcel_id` in, or None if it never placed it."""
        for slot_id, what in self.occupied.items():
            if what == parcel_id:
                return slot_id
        return None

    def summary(self) -> str:
        """Short text for the dashboard: filled / blocked slots."""
        if not self.occupied:
            return "all slots free"
        return " ".join(f"{k}={v}" for k, v in sorted(self.occupied.items()))
