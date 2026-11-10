"""moving_obstacles.py — supervisor-driven moving obstacles (environment, not robot).

Every mover is KINEMATIC: the supervisor computes its pose and writes
``translation`` / ``rotation`` once per step. Earlier versions forced a
velocity with setVelocity() on a Physics body every step, which caused two
failures seen in the logs:

  * unlimited force: the obstacle shoved / twisted the robot even while it was
    standing still yielding (rear-bumper hits, a 14 deg heading jump the EKF
    then rejected) and rammed the static crate OBST_3 on every pass;
  * ODE auto-disable: once commanded to 0 the body fell asleep, later
    setVelocity() calls no longer moved it (getVelocity() read +-0.25 m/s while
    getPosition() stayed frozen) and the robot deadlocked behind it.

A kinematic Solid has neither problem: it cannot be pushed off its lane, it
never sleeps, and it only goes where the state machine below sends it. The
robot still collides with it (boundingObject), sees it on LiDAR / cameras and
has to yield or replan exactly as before; it is never told the mover's state.
Ground truth (the robot's position) is used ONLY to animate the movers.

State machine (LaneMover):
    DRIVE    move along the lane, facing the direction of travel
    TURN     at a lane end: turn round in place (DecoyBot) / just reverse (carts)
    YIELD    IntelliBot ahead within yield_dist -> stop (beacon flashes)
    BACKOFF  stopped > max_yield s and the robot is still there -> back up
    HOLD     backed up: wait until the robot has gone (<= hold_max s)
"""

import math

import config

DRIVE, TURN, YIELD, BACKOFF, HOLD = "DRIVE", "TURN", "YIELD", "BACKOFF", "HOLD"


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _has_physics(node):
    try:
        return node.getField("physics").getSFNode() is not None
    except Exception:
        return False


class LaneMover:
    """Shuttles one Solid between two points of a straight lane (x or y)."""

    def __init__(self, node, axis, lo, hi, speed, radius, half_len, half_wid,
                 yield_dist=0.45, max_yield=4.0, backoff_dist=0.45, hold_max=20.0,
                 turn_rate=None, name="", dt=None, log=print):
        self.node = node
        self.axis = axis                      # 0 = lane along x, 1 = along y
        self.lo, self.hi = min(lo, hi), max(lo, hi)
        self.speed = speed
        self.radius = radius                  # footprint radius for the gap test
        self.half_len, self.half_wid = half_len, half_wid
        self.yield_dist = yield_dist
        self.max_yield = max_yield
        self.backoff_dist = backoff_dist
        self.hold_max = hold_max
        self.turn_rate = turn_rate            # None: carts reverse without turning
        self.name = name
        self.log = log
        self.dt = (config.BASIC_TIME_STEP_MS / 1000.0) if dt is None else dt
        self._tr = node.getField("translation")
        self._rot = node.getField("rotation")
        self._physics = _has_physics(node)
        x, y, z = self._tr.getSFVec3f()       # TypeError if the node is unreadable
        self.z = z
        self.s = min(max((x, y)[axis], self.lo), self.hi)    # along-lane position
        self.c = (x, y)[1 - axis]                             # lane centre line
        self.direction = 1.0 if self.s < self.hi else -1.0
        self.heading_of = lambda d: (0.0 if axis == 0 else math.pi / 2) + (0.0 if d > 0 else math.pi)
        r = self._rot.getSFRotation()
        yaw0 = r[3] * (1.0 if r[2] >= 0 else -1.0) if abs(r[2]) > 0.5 else 0.0
        # carts keep their world yaw for ever (they shuttle without turning);
        # the decoy starts facing its direction of travel
        self.yaw = yaw0 if self.turn_rate is None else self.heading_of(self.direction)
        self.state = DRIVE
        self.t_state = 0.0
        self.backed = 0.0
        self.v = 0.0                          # signed speed along the body's forward axis
        self.w = 0.0                          # yaw rate
        self.events = {"yields": 0, "backoffs": 0, "turns": 0}
        self._write()

    # ------------------------------------------------------------- geometry
    def _robot_rel(self, robot_xyz):
        """(along, cross) offset of the robot from the mover centre."""
        p = (robot_xyz[0], robot_xyz[1])
        return p[self.axis] - self.s, p[1 - self.axis] - self.c

    def _gap(self, along, cross):
        return math.hypot(along, cross) - self.radius - config.ROBOT_RADIUS

    def _robot_ahead(self, robot_xyz, direction, dist):
        """Robot within `dist` (surface gap) on the side we move towards."""
        if robot_xyz is None:
            return False
        along, cross = self._robot_rel(robot_xyz)
        if along * direction <= -0.05:
            return False
        if abs(cross) > self.half_wid + config.ROBOT_RADIUS + dist + 0.2:
            return False
        return self._gap(along, cross) < dist

    def _robot_close(self, robot_xyz, dist):
        if robot_xyz is None:
            return False
        return self._gap(*self._robot_rel(robot_xyz)) < dist

    # ---------------------------------------------------------------- state
    def _set(self, state):
        if state != self.state:
            self.state = state
            self.t_state = 0.0
            if state == YIELD:
                self.events["yields"] += 1
                self.log(f"[MOVER] {self.name} stops to give way to IntelliBot")
            elif state == BACKOFF:
                self.events["backoffs"] += 1
                self.log(f"[MOVER] {self.name} still blocked after {self.max_yield:.0f} s "
                         f"— backing up to clear the way")
            elif state == TURN:
                self.events["turns"] += 1

    def step(self, robot_xyz=None):
        dt = self.dt
        self.t_state += dt
        self.v, self.w = 0.0, 0.0
        d = self.direction
        if self.state == DRIVE:
            if self._robot_ahead(robot_xyz, d, self.yield_dist):
                self._set(YIELD)
            elif (d > 0 and self.s >= self.hi) or (d < 0 and self.s <= self.lo):
                self.direction = -d
                self._set(TURN if self.turn_rate else DRIVE)
            else:
                self._move(d * self.speed)
        elif self.state == TURN:
            target = self.heading_of(self.direction)
            err = _wrap(target - self.yaw)
            if self._robot_close(robot_xyz, 0.12):
                pass                                    # do not swing into the robot
            elif abs(err) < math.radians(1.0):
                self.yaw = target
                self._set(DRIVE)
            else:
                step = max(-self.turn_rate * dt, min(self.turn_rate * dt, err))
                self.yaw = _wrap(self.yaw + step)
                self.w = step / dt
        elif self.state == YIELD:
            if not self._robot_ahead(robot_xyz, d, self.yield_dist + 0.10):
                self._set(DRIVE)
            elif self.t_state >= self.max_yield:
                self.backed = 0.0
                self._set(BACKOFF)
        elif self.state == BACKOFF:
            room = (self.s - self.lo) if d > 0 else (self.hi - self.s)
            clear = not self._robot_ahead(robot_xyz, d, self.yield_dist + 0.35)
            behind_blocked = self._robot_ahead(robot_xyz, -d, 0.15)
            if clear or self.backed >= self.backoff_dist or room <= 0.0 or behind_blocked:
                self._set(HOLD)
            else:
                v = -d * 0.6 * self.speed
                self._move(v)
                self.backed += abs(v) * dt
        elif self.state == HOLD:
            if not self._robot_ahead(robot_xyz, d, self.yield_dist + 0.35) and self.t_state > 1.0:
                self._set(DRIVE)
            elif self.t_state >= self.hold_max:
                self.direction = -d                     # give up this way: go back
                self._set(TURN if self.turn_rate else DRIVE)
        self._write()
        self._animate()

    def _move(self, v_lane):
        """Advance along the lane by v_lane * dt (clamped to the lane)."""
        s_new = min(max(self.s + v_lane * self.dt, self.lo), self.hi)
        moved = s_new - self.s
        self.s = s_new
        fwd = math.cos(self.yaw) if self.axis == 0 else math.sin(self.yaw)
        self.v = (moved / self.dt) * (1.0 if fwd >= 0 else -1.0)

    def _write(self):
        p = [0.0, 0.0, self.z]
        p[self.axis], p[1 - self.axis] = self.s, self.c
        self._tr.setSFVec3f(p)
        self._rot.setSFRotation([0.0, 0.0, 1.0, self.yaw])
        if self._physics:
            self.node.resetPhysics()            # (v2 world) never let ODE fight the pose

    def _animate(self):
        pass

    def summary(self):
        return dict(self.events, state=self.state)


class DecoyBot(LaneMover):
    """Yellow look-alike robot: spinning wheels + beacon that flashes while it
    gives way. Purely cosmetic — no devices, no controller, no picking."""

    WHEEL_RADIUS = 0.05
    WHEEL_TRACK = 0.23                          # m between the wheel centres

    def __init__(self, robot, node, **kw):
        (x0, y0), (x1, y1) = config.DECOY_LANE
        axis = 0 if abs(x1 - x0) > abs(y1 - y0) else 1
        lo, hi = (x0, x1) if axis == 0 else (y0, y1)
        length, width, _h = config.DECOY_BODY
        super().__init__(node, axis, lo, hi, config.DECOY_SPEED, config.DECOY_RADIUS,
                         length / 2, width / 2,
                         yield_dist=config.DECOY_YIELD_DIST,
                         max_yield=config.DECOY_MAX_YIELD,
                         backoff_dist=config.DECOY_BACKOFF_DIST,
                         hold_max=config.DECOY_HOLD_MAX,
                         turn_rate=config.DECOY_TURN_RATE, name=config.DECOY_DEF, **kw)
        base = config.DECOY_DEF
        self._wheels = []
        for side, sign in (("L", 1.0), ("R", -1.0)):
            n = robot.getFromDef(f"{base}_WHEEL_{side}")
            if n is not None:
                self._wheels.append([n.getField("rotation"), sign, 0.0])
        beacon = robot.getFromDef(f"{base}_BEACON")
        self._beacon = beacon.getField("emissiveColor") if beacon is not None else None
        self._beacon_on = None
        self._t = 0.0

    def _animate(self):
        self._t += self.dt
        for w in self._wheels:
            field, sign, ang = w
            rate = (self.v - sign * self.w * self.WHEEL_TRACK / 2) / self.WHEEL_RADIUS
            if abs(rate) > 1e-6:
                w[2] = _wrap(ang + rate * self.dt)
                field.setSFRotation([0.0, 1.0, 0.0, w[2]])
        if self._beacon is not None:
            waiting = self.state in (YIELD, BACKOFF, HOLD)
            # steady dim amber while driving, 2 Hz bright flash while giving way
            on = (int(self._t * 4) % 2 == 0) if waiting else None
            if on != self._beacon_on:
                self._beacon_on = on
                col = [1.0, 0.55, 0.0] if on else ([0.2, 0.1, 0.0] if waiting else [0.45, 0.25, 0.0])
                self._beacon.setSFColor(col)


def build_movers(robot, log=print):
    """All moving obstacles present in the loaded world. A mover whose node
    cannot be read is skipped with a warning (never kills the controller)."""
    movers = []

    def add(name, make):
        try:
            movers.append(make())
        except Exception as exc:
            log(f"[MOVER] {name}: not animated ({type(exc).__name__}: {exc})")

    for def_name, spec in getattr(config, "CART_DEFS", {}).items():
        node = robot.getFromDef(def_name)
        lane = config.MOVING_OBSTACLE_LANES.get(spec["lane"])
        if node is None or lane is None:
            continue
        ax = spec["axis"]
        try:
            size = node.getField("boundingObject").getSFNode().getField("size").getSFVec3f()
            size = tuple(float(v) for v in size)
        except Exception:
            size = (0.45, 0.4, 0.3)
        add(def_name, lambda: LaneMover(node, ax, lane[0][ax], lane[1][ax], spec["speed"],
                                        radius=math.hypot(size[0], size[1]) / 2,
                                        half_len=size[ax] / 2, half_wid=size[1 - ax] / 2,
                                        name=def_name, log=log))
    decoy = robot.getFromDef(getattr(config, "DECOY_DEF", "DECOY_BOT"))
    if decoy is not None:
        add(config.DECOY_DEF, lambda: DecoyBot(robot, decoy, log=log))
    return movers
