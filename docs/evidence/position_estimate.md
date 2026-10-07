# Package position estimate (checklist 4.9)

Offline evaluation of the §10.8 chain on the spin frames with the
PINHOLE fallback range (d = f_px·PKG_SIZE/bbox_width) — no per-frame
LiDAR logs exist for these frames. The LiDAR-fused estimate is
printed by the controller during FIND_PACKAGE (live number).

| frame | pkg | range est (m) | bearing (deg) | est (x, y) | error (m) |
|---|---|---|---|---|---|
| spin_150.png | P1 | 2.34 | +15.8 | (-2.27, 0.27) | 0.86 |
| spin_150.png | P2 | 2.25 | +25.3 | (-2.25, -0.12) | 0.86 |
| spin_180.png | P1 | 2.34 | -15.6 | (-2.26, 0.33) | 0.86 |
| spin_180.png | P2 | 2.79 | -6.2 | (-2.77, 0.00) | 0.33 |
| spin_180.png | P3 | 2.79 | +3.5 | (-2.78, -0.47) | 0.32 |
| spin_210.png | P3 | 3.45 | -27.3 | (-3.44, -0.46) | 0.34 |

- mean error: **0.59 m**
- max error: **0.86 m**
- note: at ~3.1 m the pinhole range is quantization-limited (±1 px bbox ≈ ±5 % range). The live LiDAR-fused estimate is expected to be clearly better; record it from the console during the demo run.
