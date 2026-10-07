# LiDAR validation (Phases 4.2/4.4)

Source: `logs\lidar_raw.csv` (1321 scans, 360 rays), calibrated against the known map.

**Ray order: LIDAR_ANGLE_SIGN = -1, LIDAR_ANGLE_OFFSET = 0.020 rad** (median |measured − expected| = 8.0 cm)

| expected range (m) | rays | mean err (cm) | mean |err| (cm) |
|---|---|---|---|
| 0.5 | 69499 | +32.32 | 34.06 |
| 1.0 | 45168 | +98.28 | 98.38 |
| 1.5 | 64365 | +42.79 | 42.84 |
| 2.0 | 181527 | +3.67 | 17.12 |

Set the values above in `config.py`, then re-run the mission;
world-frame hits (calibrated) saved to `logs/lidar_scan.csv`
for `tools/plot_lidar.py`.
