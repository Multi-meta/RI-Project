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
| World file generated from config (`worlds/warehouse.wbt`) | generated; **opens-clean check pending (needs Webots)** |

## Assumed — verify at first Webots run (Phase 0.1 / 4.2 / 3.2)

1. **Header**: `#VRML_SIM R2025a` written by `tools/generate_world.py`.
   If Webots complains, copy the header from any shipped world.
2. **Coordinate system**: `WorldInfo.coordinateSystem "ENU"` is set. Expect:
   x east, y north, z up; robot yaw from `InertialUnit.getRollPitchYaw()[2]`,
   0 = facing +x, CCW positive. **Verify** with drive_test: forward should
   increase x, spin-left should increase yaw.
3. **LiDAR ray order**: config assumes ray 0 at −fov/2 (rightmost), index
   increasing CCW (`LIDAR_ANGLE_SIGN=+1`, offset 0). **Verify** per checklist
   4.2: place an obstacle on the LEFT only; short indices must map to +90°.
   If reversed, set `LIDAR_ANGLE_SIGN=-1` in config.py.
4. **Camera**: BGRA bytes, row-major; horizontal FOV field. Verify saved
   frames are not flipped/blue-shifted (checklist 4.5).
5. **Motors**: velocity mode `setPosition(float('inf'))` then `setVelocity()`;
   respect `getMaxVelocity()` (config max 12 rad/s).
6. **First sensor step**: PositionSensor may return NaN on step 1 —
   `Odometry.update` drops NaN readings (unit-tested).
7. **Physics stability**: robot rests on 2 wheels (r=0.05) + 2 zero-friction
   caster spheres; body bounding box lifted to z=0.08. If it jitters, lower
   `basicTimeStep` to 8 ms or raise body mass in config.py.
8. **EXTERNPROTO**: avoided entirely — the world uses only primitive nodes
   (Solid/Shape/Box/Cylinder/Sphere/Lidar/Camera/GPS/InertialUnit/Robot), so
   no PROTO headers are needed.
9. **Lighting/HSV**: one directional light, `castShadows FALSE`,
   ambientIntensity 0.7 — shadows must stay OFF to keep HSV detection stable.
