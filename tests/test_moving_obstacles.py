"""Kinematic moving obstacles: the yellow DECOY_BOT and the carts.

Fake Webots nodes (fields only) — the movers are pure pose bookkeeping, so the
lane, give-way and back-up logic is tested without Webots.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "controllers",
                                "intellibot_controller"))

import config  # noqa: E402
from moving_obstacles import (BACKOFF, DRIVE, HOLD, TURN, YIELD,  # noqa: E402
                              DecoyBot, LaneMover, build_movers)


class Field:
    def __init__(self, v=None):
        self.v = v

    def getSFVec3f(self):
        return list(self.v)

    def setSFVec3f(self, v):
        self.v = list(v)

    def getSFRotation(self):
        return list(self.v)

    def setSFRotation(self, v):
        self.v = list(v)

    def setSFColor(self, v):
        self.v = list(v)

    def getSFNode(self):
        return self.v


class Node:
    def __init__(self, pos=(0, 0, 0), rot=(0, 0, 1, 0), physics=None):
        self.fields = {"translation": Field(pos), "rotation": Field(rot),
                       "physics": Field(physics), "emissiveColor": Field([0, 0, 0])}
        self.resets = 0

    def getField(self, name):
        return self.fields[name]

    def resetPhysics(self):
        self.resets += 1

    def pos(self):
        return self.fields["translation"].v


class World:
    def __init__(self):
        x0, y0 = config.DECOY_LANE[0]
        self.nodes = {config.DECOY_DEF: Node((x0, y0, 0.0)),
                      f"{config.DECOY_DEF}_WHEEL_L": Node(),
                      f"{config.DECOY_DEF}_WHEEL_R": Node(),
                      f"{config.DECOY_DEF}_BEACON": Node()}

    def getFromDef(self, name):
        return self.nodes.get(name)


def make_decoy():
    w = World()
    return w, DecoyBot(w, w.nodes[config.DECOY_DEF], log=lambda *_: None)


def run(m, robot=None, seconds=1.0):
    for _ in range(int(seconds / m.dt)):
        m.step(robot)


def test_decoy_shuttles_whole_lane_and_turns_round_at_the_ends():
    _w, m = make_decoy()
    (_, y0), (_, y1) = config.DECOY_LANE
    assert m.state == DRIVE and math.isclose(m.yaw, math.pi / 2, abs_tol=1e-6)
    seen_turn = False
    ys = []
    for _ in range(int(2.5 * (y1 - y0) / config.DECOY_SPEED / m.dt)):
        m.step(None)
        seen_turn |= m.state == TURN
        ys.append(m.node.pos()[1])
    assert seen_turn
    assert max(ys) <= y1 + 1e-9 and min(ys) >= y0 - 1e-9
    assert max(ys) > y1 - 0.01                       # reached the far end
    assert m.node.pos()[0] == config.DECOY_LANE[0][0]   # never leaves the lane
    assert m.events["turns"] >= 1


def test_decoy_stops_for_the_robot_and_never_moves_towards_it():
    _w, m = make_decoy()
    run(m, None, 1.0)                                # driving +y
    y = m.node.pos()[1]
    robot = (m.node.pos()[0], y + config.DECOY_RADIUS + config.ROBOT_RADIUS + 0.3, 0.0)
    m.step(robot)
    assert m.state == YIELD
    y_stop = m.node.pos()[1]
    run(m, robot, config.DECOY_MAX_YIELD - 0.2)
    assert m.state == YIELD and m.node.pos()[1] == y_stop   # waits in place
    run(m, robot, 0.5)
    assert m.state in (BACKOFF, HOLD)                # gives way by backing up
    run(m, robot, 3.0)
    assert m.node.pos()[1] < y_stop                  # moved AWAY from the robot
    assert y_stop - m.node.pos()[1] <= config.DECOY_BACKOFF_DIST + 0.01
    assert m.events["yields"] == 1 and m.events["backoffs"] == 1


def test_decoy_resumes_once_the_robot_has_passed():
    _w, m = make_decoy()
    run(m, None, 1.0)
    y = m.node.pos()[1]
    robot = (m.node.pos()[0], y + 0.6, 0.0)
    m.step(robot)
    assert m.state == YIELD
    run(m, (5.0, 5.0, 0.0), 0.2)                     # robot gone
    assert m.state == DRIVE
    y_before = m.node.pos()[1]
    run(m, None, 0.5)
    assert m.node.pos()[1] > y_before


def test_decoy_ignores_a_robot_behind_it():
    _w, m = make_decoy()
    run(m, None, 1.0)
    y = m.node.pos()[1]
    run(m, (m.node.pos()[0], y - 0.45, 0.0), 0.5)    # robot behind, close
    assert m.state == DRIVE and m.node.pos()[1] > y


def test_decoy_animates_wheels_and_flashes_beacon_while_giving_way():
    w, m = make_decoy()
    run(m, None, 0.5)
    wl = w.nodes[f"{config.DECOY_DEF}_WHEEL_L"].fields["rotation"].v
    assert wl[:3] == [0.0, 1.0, 0.0] and abs(wl[3]) > 0.1
    beacon = w.nodes[f"{config.DECOY_DEF}_BEACON"].fields["emissiveColor"]
    driving = list(beacon.v)
    y = m.node.pos()[1]
    robot = (m.node.pos()[0], y + 0.6, 0.0)
    colours = set()
    for _ in range(int(1.0 / m.dt)):
        m.step(robot)
        colours.add(tuple(beacon.v))
    assert len(colours) == 2 and tuple(driving) not in colours   # flashing


def test_cart_keeps_its_yaw_and_shuttles_without_turning():
    node = Node((0.72, -2.4, 0.15))
    m = LaneMover(node, 1, -2.4, 2.4, 0.3, radius=0.28, half_len=0.2, half_wid=0.2,
                  name="CART_2", log=lambda *_: None)
    run(m, None, 20.0)
    assert node.fields["rotation"].v == [0.0, 0.0, 1.0, 0.0]
    assert node.pos()[0] == 0.72 and node.pos()[2] == 0.15
    assert m.events["turns"] == 0 and m.direction == -1.0       # reversed at the end


def test_build_movers_skips_unreadable_nodes():
    class Bad:
        def getFromDef(self, name):
            return Node(pos=None) if name == config.DECOY_DEF else None

    msgs = []
    assert build_movers(Bad(), log=msgs.append) == []
    assert any("not animated" in s for s in msgs)
