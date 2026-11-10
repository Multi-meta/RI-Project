"""fault_injector.py — inject environment and sensor faults to test sensor fusion.

Two kinds of faults, both seen by the robot ONLY through degraded sensor data
(the navigation stack is never told that a fault is active):

Environment (Supervisor edits the world, so the real renderer is affected):
    lights_dim    [factor=0.25]  both DirectionalLights + Background scaled down
    lights_off    [-]            lights switched off (cameras go dark)
    smoke         [vis=1.5 m]    Webots Fog with that visibility range (cameras),
                                 plus LiDAR attenuation modelled in software
                                 (Webots LiDARs do not see Fog): long rays are
                                 absorbed, some rays return nothing, a few
                                 return spurious short back-scatter echoes.
    floodlight    [intensity=40] very strong overhead PointLight: the light
                                 sensor saturates (GLARE), cameras wash out,
                                 and the IR LiDAR loses its weak far echoes in
                                 the ambient IR (software model). Ultrasound
                                 is unaffected -> range fusion leans on it.
    foggy_night   [vis=1.2 m]    lights at NIGHT_LIGHT_SCALE + dense Fog +
                                 strong LiDAR attenuation / back-scatter.
                                 Ultrasound is unaffected.

Sensor (applied to the readings in the controller loop):
    gps_dropout   [-]            GPS returns NaN          -> EKF dead-reckons
    gps_bias      [m=1.0]        GPS offset in +x          -> EKF gate rejects
    gps_noise     [sigma=0.5 m]  extra white GPS noise     -> R inflation
    imu_dropout   [-]            IMU yaw NaN               -> odometry heading
    imu_bias      [deg=20]       IMU yaw offset            -> gate / drift
    lidar_dropout [-]            all rays invalid (NaN)    -> LiDAR FAILED, stop
    lidar_sector  [deg=90]       front sector invalid      -> LiDAR DEGRADED
    wheel_slip    [frac=0.3]     encoders over-count       -> GPS corrects drift
    camera_blind  [front|rear|both] black frames           -> camera BLANK

How to inject
    1) Schedule (batch / repeatable): environment variable
         INTELLIBOT_FAULTS="lights_off@30+40,smoke@80+40:1.2,gps_dropout@130+25"
       i.e. type@start_s[+duration_s][:param], comma separated. No duration =
       until the end. config.FAULT_SCHEDULE is used when the variable is unset.
    2) Live (Webots GUI): click the 3D view, then press
         L  lights: normal -> dim -> off -> normal
         K  smoke on/off          G  GPS dropout       B  GPS bias
         I  IMU bias              J  LiDAR dropout     F  LiDAR front sector
         W  wheel slip            C  front camera blind
         H  floodlight on/off     N  foggy night on/off
         0  clear every fault
Every activation / clearance is printed ([FAULT] ...), logged in run.csv
("faults" column) and summarised in the mission report with how the fusion
reacted (localisation status, EKF error vs ground truth, LiDAR/camera health).
"""

import math
import os

import numpy as np

import config

ENV_FAULTS = ("lights_dim", "lights_off", "smoke", "floodlight", "foggy_night")
SENSOR_FAULTS = ("gps_dropout", "gps_bias", "gps_noise", "imu_dropout", "imu_bias",
                 "lidar_dropout", "lidar_sector", "wheel_slip", "camera_blind")
FAULT_TYPES = ENV_FAULTS + SENSOR_FAULTS
DEFAULT_PARAM = {"lights_dim": 0.25, "smoke": 1.5, "gps_bias": 1.0, "gps_noise": 0.5,
                 "imu_bias": 20.0, "lidar_sector": 90.0, "wheel_slip": 0.3,
                 "camera_blind": "front",
                 "floodlight": getattr(config, "FLOODLIGHT_INTENSITY", 40.0),
                 "foggy_night": getattr(config, "FOG_VISIBILITY", 1.2)}


def parse_schedule(spec):
    """'type@start[+dur][:param],...' -> [dict(type, start, end, param)]."""
    out = []
    for item in (spec or "").replace(";", ",").split(","):
        item = item.strip()
        if not item:
            continue
        param = None
        if ":" in item:
            item, param = item.split(":", 1)
        if "@" not in item:
            raise ValueError(f"fault '{item}': expected type@start[+duration][:param]")
        ftype, when = item.split("@", 1)
        ftype = ftype.strip().lower()
        if ftype not in FAULT_TYPES:
            raise ValueError(f"unknown fault type '{ftype}' (one of {', '.join(FAULT_TYPES)})")
        if "+" in when:
            start, dur = when.split("+", 1)
            start, end = float(start), float(start) + float(dur)
        else:
            start, end = float(when), math.inf
        if param is None:
            param = DEFAULT_PARAM.get(ftype)
        elif ftype != "camera_blind":
            param = float(param)
        out.append(dict(type=ftype, start=start, end=end, param=param))
    return out


class FaultInjector:
    HOTKEYS = {"G": "gps_dropout", "B": "gps_bias", "I": "imu_bias",
               "J": "lidar_dropout", "F": "lidar_sector", "W": "wheel_slip",
               "C": "camera_blind", "K": "smoke", "H": "floodlight",
               "N": "foggy_night"}

    def __init__(self, robot, timestep, spec=None, log=print):
        self.robot = robot
        self.log = log
        if spec is None:
            spec = os.environ.get("INTELLIBOT_FAULTS", getattr(config, "FAULT_SCHEDULE", ""))
        try:
            self.schedule = parse_schedule(spec)
        except ValueError as exc:
            self.log(f"[FAULT] bad schedule ignored: {exc}")
            self.schedule = []
        self.active = {}                 # type -> param (scheduled or manual)
        self.manual = {}                 # type -> param (hotkeys)
        self.events = []                 # report rows
        self._open = {}                  # type -> report row while active
        self.rng = np.random.default_rng(getattr(config, "RANDOM_SEED", 0))
        self._env_applied = {}
        self._lights = []                # (node, intensity, ambient, on)
        for d in getattr(config, "LIGHT_DEFS", ("DIRLIGHT_1", "DIRLIGHT_2")):
            n = robot.getFromDef(d)
            if n is not None:
                self._lights.append((n, n.getField("intensity").getSFFloat(),
                                     n.getField("ambientIntensity").getSFFloat(),
                                     n.getField("on").getSFBool()))
        self._bg = robot.getFromDef(getattr(config, "BACKGROUND_DEF", "BACKGROUND"))
        self._bg_lum = (self._bg.getField("luminosity").getSFFloat()
                        if self._bg is not None else None)
        # the low cameras mostly see the BACKGROUND colour, which no light or
        # luminosity affects: an indoor warehouse has no sky, so the light
        # faults scale this colour too (otherwise "lights off" leaves the
        # camera at ~70 grey and nothing degrades)
        self._sky = None
        if self._bg is not None:
            try:
                f = self._bg.getField("skyColor")
                self._sky = [list(f.getMFColor(i)) for i in range(int(f.getCount()))]
            except Exception:
                self._sky = None
        self._fog = robot.getFromDef(getattr(config, "SMOKE_FOG_DEF", "SMOKE"))
        self._flood = robot.getFromDef(getattr(config, "FLOODLIGHT_DEF", "FLOODLIGHT"))
        self._lamp = robot.getFromDef(getattr(config, "FLOODLAMP_DEF", "FLOODLAMP"))
        self._light_mode = 0             # hotkey L cycle: 0 normal, 1 dim, 2 off
        self.keyboard = None
        if getattr(config, "FAULT_HOTKEYS", True):
            try:
                self.keyboard = robot.getKeyboard()
                self.keyboard.enable(timestep)
            except Exception:
                self.keyboard = None
        if self.schedule:
            self.log("[FAULT] schedule: " + ", ".join(
                f"{f['type']}@{f['start']:g}s"
                + ("" if math.isinf(f['end']) else f"-{f['end']:g}s")
                + (f" ({f['param']})" if f['param'] is not None else "")
                for f in self.schedule))
        if self.keyboard is not None:
            self.log("[FAULT] live hotkeys (click the 3D view): L lights, K smoke, "
                     "G GPS drop, B GPS bias, I IMU bias, J LiDAR drop, F LiDAR sector, "
                     "W wheel slip, C camera blind, H floodlight, N foggy night, 0 clear")

    # ------------------------------------------------------------ schedule
    def update(self, t):
        self._read_keys(t)
        want = dict(self.manual)
        for f in self.schedule:
            if f["start"] <= t < f["end"]:
                want[f["type"]] = f["param"]
        for ftype in list(self.active):
            if ftype not in want:
                self._deactivate(ftype, t)
        for ftype, param in want.items():
            if self.active.get(ftype, object()) != param:
                self._activate(ftype, param, t)

    def _read_keys(self, t):
        if self.keyboard is None:
            return
        key = self.keyboard.getKey()
        while key != -1:
            ch = chr(key & 0xFF).upper() if (key & 0xFF) else ""
            if ch == "L":
                self._light_mode = (self._light_mode + 1) % 3
                self.manual.pop("lights_dim", None)
                self.manual.pop("lights_off", None)
                if self._light_mode == 1:
                    self.manual["lights_dim"] = DEFAULT_PARAM["lights_dim"]
                elif self._light_mode == 2:
                    self.manual["lights_off"] = None
            elif ch == "0":
                self.manual.clear()
                self._light_mode = 0
                self.schedule = [f for f in self.schedule if f["start"] > t]
            elif ch in self.HOTKEYS:
                ftype = self.HOTKEYS[ch]
                if ftype in self.manual:
                    del self.manual[ftype]
                else:
                    self.manual[ftype] = DEFAULT_PARAM.get(ftype)
            key = self.keyboard.getKey()

    def _activate(self, ftype, param, t):
        self.active[ftype] = param
        self.log(f"[FAULT] [t={t:.1f}s] {ftype} ON" + (f" ({param})" if param is not None else ""))
        row = dict(type=ftype, param=param, start=round(t, 2), end=None,
                   loc_status=set(), lidar_health=set(), camera_health=set(),
                   speed_cap=set(), max_ekf_err_m=0.0, _err2=0.0, _n=0,
                   camera_min_mean=None, camera_min_std=None,
                   light_health=set(), light_max=None, range_source=set(),
                   max_w_sonar=0.0, phantoms_removed=0)
        self._open[ftype] = row
        self.events.append(row)
        self._apply_env()

    def _deactivate(self, ftype, t):
        self.active.pop(ftype, None)
        self.log(f"[FAULT] [t={t:.1f}s] {ftype} CLEARED")
        row = self._open.pop(ftype, None)
        if row is not None:
            row["end"] = round(t, 2)
        self._apply_env()

    # ---------------------------------------------------------- environment
    def _apply_env(self):
        dim = self.active.get("lights_dim")
        off = "lights_off" in self.active
        scale = 0.0 if off else (float(dim) if dim is not None else 1.0)
        if "foggy_night" in self.active:
            scale = min(scale, config.NIGHT_LIGHT_SCALE)
        if self._env_applied.get("light") != scale:
            self._env_applied["light"] = scale
            for node, inten, amb, on in self._lights:
                node.getField("on").setSFBool(on and not off)
                node.getField("intensity").setSFFloat(inten * scale)
                node.getField("ambientIntensity").setSFFloat(amb * scale)
            if self._bg is not None and self._bg_lum is not None:
                self._bg.getField("luminosity").setSFFloat(self._bg_lum * max(scale, 0.02))
            if self._sky:
                f = self._bg.getField("skyColor")
                k = max(scale, 0.03)
                for i, rgb in enumerate(self._sky):
                    f.setMFColor(i, [c * k for c in rgb])
        vis = self._fog_visibility()
        if self._env_applied.get("smoke") != vis:
            self._env_applied["smoke"] = vis
            if self._fog is not None:
                self._fog.getField("visibilityRange").setSFFloat(vis)   # 0 = no fog
            elif vis > 0:
                self.log("[FAULT] no 'DEF SMOKE Fog' in this world — smoke affects "
                         "the LiDAR only (regenerate the world for camera smoke)")
        flood = self.active.get("floodlight")
        if self._env_applied.get("flood", "unset") != flood:
            self._env_applied["flood"] = flood
            if self._flood is not None:
                self._flood.getField("on").setSFBool(flood is not None)
                if flood is not None:
                    self._flood.getField("intensity").setSFFloat(float(flood))
                if self._lamp is not None:
                    self._lamp.getField("emissiveColor").setSFColor(
                        [1.0, 1.0, 0.92] if flood is not None else [0.0, 0.0, 0.0])
            elif flood is not None:
                self.log("[FAULT] no 'DEF FLOODLIGHT PointLight' in this world — "
                         "floodlight affects the LiDAR model only (regenerate the world)")

    def _fog_visibility(self):
        """Smallest active fog/smoke visibility (m); 0 = clear air."""
        vis = [float(self.active[k]) for k in ("smoke", "foggy_night")
               if self.active.get(k) is not None]
        return min(vis) if vis else 0.0

    # -------------------------------------------------------------- sensors
    def apply_gps(self, xy):
        if xy is None:
            return xy
        if "gps_dropout" in self.active:
            return (math.nan, math.nan)
        x, y = xy
        if "gps_bias" in self.active:
            x += float(self.active["gps_bias"])
        if "gps_noise" in self.active:
            s = float(self.active["gps_noise"])
            x += self.rng.normal(0.0, s)
            y += self.rng.normal(0.0, s)
        return (x, y)

    def apply_imu(self, yaw):
        if yaw is None:
            return yaw
        if "imu_dropout" in self.active:
            return math.nan
        if "imu_bias" in self.active:
            yaw = math.atan2(math.sin(yaw + math.radians(float(self.active["imu_bias"]))),
                             math.cos(yaw + math.radians(float(self.active["imu_bias"]))))
        return yaw

    def apply_odometry(self, ds, dth):
        """Wheel slip: the LEFT wheel spins (1+frac) times what the robot rolls,
        so odometry over-counts distance and drifts in heading."""
        if "wheel_slip" not in self.active or ds is None:
            return ds, dth
        p = float(self.active["wheel_slip"])
        extra = p * ds / 2.0                       # left-wheel excess, projected
        return ds + extra, dth - 2.0 * extra / config.WHEEL_SEPARATION

    def apply_lidar(self, scan, angles):
        """-> scan with INVALID rays as NaN (what a real driver reports)."""
        if not any(k in self.active for k in ("lidar_dropout", "lidar_sector", "smoke",
                                               "floodlight", "foggy_night")):
            return scan
        r = np.array(scan, dtype=float, copy=True)
        if "lidar_dropout" in self.active:
            r[:] = math.nan
            return r
        if "lidar_sector" in self.active:
            half = math.radians(float(self.active["lidar_sector"])) / 2.0
            a = np.asarray(angles, dtype=float)
            r[np.abs(np.arctan2(np.sin(a), np.cos(a))) <= half] = math.nan
        n = len(r)
        if "floodlight" in self.active:
            # ambient IR swamps the weak far echoes (flagged invalid by the driver)
            far = np.isfinite(r) & (r > config.FLOOD_LIDAR_RANGE)
            r[far & (self.rng.random(n) < config.FLOOD_LIDAR_DROP)] = math.nan
            ok = np.isfinite(r)
            r[ok] += self.rng.normal(0.0, config.FLOOD_LIDAR_NOISE, int(ok.sum()))
        if "smoke" in self.active or "foggy_night" in self.active:
            vis = max(self._fog_visibility(), 0.2)
            night = "foggy_night" in self.active
            drop = config.FOG_LIDAR_DROP if night else config.SMOKE_LIDAR_DROP
            back = config.FOG_LIDAR_BACKSCATTER if night else config.SMOKE_LIDAR_BACKSCATTER
            noise = config.FOG_LIDAR_NOISE if night else config.SMOKE_LIDAR_NOISE
            far = np.isfinite(r) & (r > 1.5 * vis)
            r[far & (self.rng.random(n) < 0.7)] = math.inf          # absorbed: no echo
            r[self.rng.random(n) < drop] = math.nan                  # weak echo, flagged
            bs = self.rng.random(n) < back
            if bs.any():                                            # back-scatter
                r[bs] = self.rng.uniform(0.35, max(0.4, vis), int(bs.sum()))
            ok = np.isfinite(r)
            r[ok] += self.rng.normal(0.0, noise, int(ok.sum()))
        return r

    def apply_cameras(self, frames):
        if "camera_blind" not in self.active:
            return frames
        from camera_pipeline import CameraFrame, frame_stats
        which = str(self.active["camera_blind"] or "front")
        roles = ("front", "rear") if which == "both" else (which,)
        for role in roles:
            fr = frames.get(role)
            if fr is None or fr.image is None:
                continue
            black = np.zeros_like(fr.image)
            frames[role] = CameraFrame(role, black, fr.t, True, "BLANK",
                                       "lens covered (fault)", frame_stats(black))
        return frames

    # --------------------------------------------------------------- report
    def observe(self, loc_status=None, ekf_err=None, lidar_health=None,
                camera_health=None, speed_cap=None, camera_stats=None,
                light_health=None, light_level=None, range_source=None,
                w_sonar=None, phantoms=0):
        """Record how the fusion reacts while each fault is active.
        camera_stats: {"mean": grey mean, "std": grey std} of the front camera."""
        for row in self._open.values():
            if light_health:
                row["light_health"].add(light_health)
            if light_level is not None and math.isfinite(light_level):
                row["light_max"] = round(max(row["light_max"] or 0.0, light_level), 1)
            if range_source:
                row["range_source"].add(range_source)
            if w_sonar is not None:
                row["max_w_sonar"] = round(max(row["max_w_sonar"], w_sonar), 3)
            row["phantoms_removed"] += int(phantoms)
            if camera_stats:
                for key, stat in (("camera_min_mean", "mean"), ("camera_min_std", "std")):
                    v = camera_stats.get(stat)
                    if v is not None:
                        row[key] = round(v if row[key] is None else min(row[key], v), 1)
            if loc_status:
                row["loc_status"].add(loc_status)
            if lidar_health:
                row["lidar_health"].add(lidar_health)
            if camera_health:
                row["camera_health"].add(camera_health)
            if speed_cap:
                row["speed_cap"].add(speed_cap)
            if ekf_err is not None and math.isfinite(ekf_err):
                row["max_ekf_err_m"] = max(row["max_ekf_err_m"], ekf_err)
                row["_err2"] += ekf_err * ekf_err
                row["_n"] += 1

    def label(self):
        return "|".join(sorted(self.active)) if self.active else ""

    def summary(self):
        out = []
        for row in self.events:
            r = {k: v for k, v in row.items() if not k.startswith("_")}
            for k in ("loc_status", "lidar_health", "camera_health", "speed_cap",
                      "light_health", "range_source"):
                r[k] = sorted(r[k])
            r["max_ekf_err_m"] = round(r["max_ekf_err_m"], 3)
            r["rmse_ekf_err_m"] = (round(math.sqrt(row["_err2"] / row["_n"]), 3)
                                   if row["_n"] else None)
            out.append(r)
        return out


def lidar_health(scan, prev="OK", degraded_frac=None, failed_frac=None):
    """(health, invalid_fraction) from the NaN-flagged rays of one scan.

    Hysteresis: a level is entered at its threshold and left only below HALF
    of it, so a fraction hovering at the threshold (smoke) does not flap."""
    if scan is None:
        return "FAILED", 1.0
    r = np.asarray(scan, dtype=float)
    frac = float(np.mean(np.isnan(r))) if r.size else 1.0
    f_fail = config.SAFETY_LIDAR_FAILED_FRAC if failed_frac is None else failed_frac
    f_deg = config.SAFETY_LIDAR_DEGRADED_FRAC if degraded_frac is None else degraded_frac
    if frac >= f_fail or (prev == "FAILED" and frac >= f_fail / 2.0):
        return "FAILED", frac
    if frac >= f_deg or (prev in ("DEGRADED", "FAILED") and frac >= f_deg / 2.0):
        return "DEGRADED", frac
    return "OK", frac
