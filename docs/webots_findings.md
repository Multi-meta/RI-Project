# Webots Findings (measured / verified conventions)

> Rule 4 of project_summary.md: verify, don't assume. This file records what
> has been **measured on the installed version** vs. what is still
> **assumed and must be verified at runtime**.

## Environment status (2026-10-05)

- **Webots R2025a** is registered on this machine (registry key
  `HKCU\SOFTWARE\Cyberbotics\Webots-R2025a`), but the application executable
  was **not found** on C:/D:/E: — only the asset cache remains
  (`%LOCALAPPDATA%\Cyberbotics\Webots\cache`). **Webots could not be run by
  the build agent.** All simulator-dependent validation (checklist "Done
  when" items that need a live run) is handed to the human operator with
  exact steps — see `docs/demo_script.md` and README "Run" section.
- Python 3.12.3 with numpy 1.26.4, opencv 4.10.0, matplotlib 3.8.3,
  pytest 8.3.4 — all importable (the Phase 0.1 throwaway-controller check
  must still be repeated inside a Webots controller once Webots is
  reinstalled).

## Measured on Webots R2025a (2026-10-06, Phases 2–3)

Webots R2025a is installed at `D:\RI_CyberBotics\Webots`; Preferences →
Python command = `python` → Python 3.11.9 (numpy 1.26.4, opencv 4.13.0;
matplotlib only needed by `tools/`). Runs were driven from the CLI:

```
DRIVE_TEST_MODE=<mode> webots --batch --mode=fast --stdout --stderr --minimize worlds/warehouse_drive_test.wbt
```

| Item | Result |
|---|---|
| World header | `#VRML_SIM R2025a utf8` is required (the `utf8` suffix was missing before). |
| Opens clean | 0 warnings / 0 errors in the console after the fixes below. |
| World vs config (2.3) | `[WORLD CHECK] world matches config` (all packages, shelves, crates within 1 cm after 1 s of settling). |
| Coordinate frame | ENU confirmed: forward command increases x; spin-left increases IMU yaw (CCW positive); robot starts at (0.000, −0.300, 0°). |
| Rest stability (3.1) | 10 s at 0 velocity: xy drift 0.00 mm, max \|dz\| 0.28 mm, tilt 0.000°, yaw drift 0.000° (`logs/stability.csv`). |
| Kinematics (3.3) | v and ω within **−0.9 %** of command for all four cases (`docs/evidence/kinematics_validation.md`). |
| Odometry (3.4) | final drift 0.8 cm (1 m line), 2.2 cm / 4.7° (2 m square), 1.1 cm / 1.2° (arc) — `docs/evidence/odometry_vs_gps.png`. |
| Camera | overlay image upright and correctly colored (robot facing +x sees Zone A/B patches) — see `world_topdown.png`. |

### Problems found and fixed during the first runs

1. **Floor had no `boundingObject`** → every physics object would fall
   through. Floor is now a colliding box with its top face at z = 0.
2. **`contactProperties` was inside the robot's `Physics` node** (invalid;
   it belongs to `WorldInfo`). Moved to `WorldInfo` as
   `ContactProperties { material2 "caster" coulombFriction [ 0 ] }`.
3. **Casters were visual-only** → robot would rest on its axle only. They
   are now `Solid`s with a sphere `boundingObject`, `contactMaterial
   "caster"` (zero friction) and a tiny mass (a `Solid` child without
   `physics` is ignored for collisions — Webots warns about this).
4. **Effective track width was L + wheel width (−14.2 % yaw rate).** With a
   `Cylinder` wheel `boundingObject` the floor contact sits at the wheel's
   outer rim, so the measured ω was 0.858 × command ⇒ L_eff = 0.28 m
   instead of 0.24 m. Confirmed by halving the wheel width (error −7.6 %,
   L_eff = 0.26 m). **Fix:** wheel `boundingObject` = `Sphere` of the wheel
   radius ⇒ single contact point exactly at y = ±L/2 ⇒ error −0.9 %. This
   also cut square-trajectory odometry drift from 35 cm / 52° to 2.2 cm / 4.7°.
   (Remaining uniform −0.9 % on v and ω ≈ 0.45 mm of soft-contact sink on
   the 5 cm wheel radius; acceptable, not calibrated away.)
5. **`DYN_OBSTACLE` overlapped the south wall** (y = −2.85 with a 0.4 m
   box). Parked at `config.DYN_OBSTACLE_PARK` = (3.5, 2.6), free NE corner.
6. **Module-name clash:** `intellibot_controller/controller.py` shadowed the
   Webots `controller` API module (`ImportError: cannot import name
   'Supervisor'`). **Fixed:** renamed to `waypoint_follower.py` (imports,
   tests and docs updated).
7. Solids now get unique `name`s (lower-case DEF) — no duplicate-name warnings.
8. **Package faces facing the robot were too dark to detect.** With one
   directional light, faces pointing away from it got ambient light only
   (HSV value ≈ 46 < detector minimum 60–70), so the first mission run ended
   `ERROR: package never found during scan`. Diagnosed with drive_test
   `capture` mode (frames in `docs/evidence/frames/`). **Fix:** two opposed
   directional lights (no shadows) → all three packages detected at 3 m.

## First full mission run (2026-10-06, `worlds/warehouse.wbt`)

```
FIND_PACKAGE -> P3 detected at (-2.50, -0.53), range 2.51 m, bearing 26.6 deg   t=4.7 s
NAVIGATE     -> A* 10 waypoints, 2.10 m, 949 expansions                         t=4.7-20.5 s
PICK         -> placeholder, distance 0.56 m                                    t=21.5 s
DELIVER      -> A* 19 waypoints, 5.04 m, 2401 expansions                        t=21.5-61.7 s
DONE         -> MISSION REPORT time=61.7 s, path_length=7.14 m, replans=0
```

Known issue: true P3 position is (-3.10, -0.50); the estimate was 0.6 m
short. Most likely the LiDAR ray order (assumption 3 below) — the range was
read at the mirrored bearing. To verify in Phase 4.2.

## Verified by offline code (no simulator needed)

| Item | Status |
|---|---|
| Occupancy grid ↔ world transforms (round trip < res/2) | **verified by pytest** |
| A* optimality == Dijkstra on 60 random grids | **verified by pytest** |
| No-corner-cutting rule | **verified by pytest** |
| Circular inflation kernel | **verified by pytest** |
| Warehouse layout: pickup→Zone B path + two distinct routes | **verified by pytest on the real grid** |
| FSM happy path IDLE→…→DONE + mission report | **verified by pytest (simulated unicycle)** |
| HSV detector on synthetic frames; zone colors not confused | **verified by pytest** |
| Pixel→bearing→world estimation chain | **verified by pytest** |
| World file generated from config (`worlds/warehouse.wbt`) | **opens clean in Webots R2025a** (see above) |

## Assumed — verify at first Webots run (Phase 0.1 / 4.2 / 3.2)

1. ~~Header~~ — verified: `#VRML_SIM R2025a utf8` (see measured table).
2. ~~Coordinate system~~ — verified with drive_test (see table). Was:
   x east, y north, z up; robot yaw from `InertialUnit.getRollPitchYaw()[2]`,
   0 = facing +x, CCW positive. **Verify** with drive_test: forward should
   increase x, spin-left should increase yaw.
3. ~~LiDAR ray order~~ — **MEASURED 2026-10-07** (1321 scans vs the known
   map via `tools/calibrate_lidar.py`): the index increases **CLOCKWISE**
   (`LIDAR_ANGLE_SIGN=-1`, offset 0.02 rad, median |err| 8 cm). This was the
   root cause of the 0.6 m package-estimate error; after the fix the live
   estimate is (−2.99, −0.57) vs true (−3.10, −0.50) = **0.13 m**. Evidence:
   `docs/evidence/lidar_validation.md`, `lidar_scan.png`.
4. **Camera**: BGRA bytes, row-major; horizontal FOV field. Verify saved
   frames are not flipped/blue-shifted (checklist 4.5).
5. **Motors**: velocity mode `setPosition(float('inf'))` then `setVelocity()`;
   respect `getMaxVelocity()` (config max 12 rad/s).
6. **First sensor step**: PositionSensor may return NaN on step 1 —
   `Odometry.update` drops NaN readings (unit-tested).
7. ~~Physics stability~~ — verified (see table): 2 sphere-collision wheels
   + 2 zero-friction caster spheres, body bottom at z = 0.03.
8. **EXTERNPROTO**: avoided entirely — the world uses only primitive nodes
   (Solid/Shape/Box/Cylinder/Sphere/Lidar/Camera/GPS/InertialUnit/Robot), so
   no PROTO headers are needed.
9. ~~Lighting/HSV~~ — superseded by fix 8: TWO opposed directional
   lights, `castShadows FALSE`; shadows must stay OFF to keep HSV stable.

## Additions (2026-10-07)

- **Env check (0.1) built in:** both controllers now print
  `python/numpy/cv2/matplotlib` versions at startup (`print_env_versions` /
  the INTELLIBOT banner), so the Phase 0.1 console proof happens on every
  run instead of needing a throwaway controller.
- **Sample worlds copied to `worlds/`** and audited
  (`docs/samples_audit.md` → "World-file audit"): all R2025a samples use
  `#VRML_SIM R2025a utf8` + EXTERNPROTO URLs only for PROTO nodes; the
  Lidar/Camera+Recognition/LED/Display/Pen/Connector field values we should
  mirror are recorded there verbatim. Header template saved to
  `tools/world_header.txt`; a minimal floor+box world instantiated from it
  is at `worlds_output/trivial_test.wbt`.
- **LiDAR calibration pipeline (4.2/4.4) automated:** `DRIVE_TEST_MODE=lidar`
  logs raw 360-ray scans while rotating (`logs/lidar_raw.csv`), then
  `python tools/calibrate_lidar.py` determines `LIDAR_ANGLE_SIGN/_OFFSET`
  against the known map, writes the range-accuracy table
  (`docs/evidence/lidar_validation.md`) and `logs/lidar_scan.csv` →
  `docs/evidence/lidar_scan.png`. This directly targets the known issue
  above (package estimate 0.6 m short = suspected mirrored ray order).
- **`worlds_output/`** now always holds fresh copies of the generated
  worlds (the previous copy was from Oct 5, before the wheel/lighting
  fixes — stale files removed by regeneration).


## More findings (2026-10-07, CLI batch runs)

- Webots runs fine headless from the CLI:
  `DRIVE_TEST_MODE=<mode> "D:\Webots\msys64\mingw64in\webots.exe" --batch --mode=fast --stdout --stderr --minimize worlds/warehouse_drive_test.wbt`
  (Python 3.12.3 now resolves via `pythonCommand=python`).
- A second Webots instance (e.g. an open GUI window) warns "port 1234
  already in use" and can silently interfere with CLI runs — close GUI
  instances before batch testing.
- **Environment colours are part of the detector's interface**: a
  decorative ball painted saturated blue was "found" as package P3 by the
  HSV detector. Decorative objects must stay outside every HSV window
  (ball is now desaturated grey).
- `simulationQuit(0)` from the controller does not reliably terminate batch
  runs, and prints buffered at process death are lost. The controller now
  breaks its own loop 2 s after DONE/ERROR, writes a final CSV row with the
  terminal state, and flushes stdout before quitting.
