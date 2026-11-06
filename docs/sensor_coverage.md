# Sensor Coverage & Modality Redundancy Matrix

Per Checklist §10.3, this matrix confirms that Front, Left, Right, and Rear quadrants each have ≥2 independent sensing modalities.

| Direction | Primary Modality | Secondary Modality | Tertiary Modality | Verification Status |
|---|---|---|---|---|
| **Front** | LiDAR (360° plane, rays -30° to +30°) | Front Camera (180° FOV, OpenCV detector) | TouchSensor Bumper (`bumper_f`) | **PASS (3 modalities)** |
| **Rear** | Rear Camera (180° FOV, `rear_camera`) | LiDAR (rear rays 150° to 210°) | TouchSensor Bumper (`bumper_r`) | **PASS (3 modalities)** |
| **Left** | LiDAR (60° to 120° sector) | 180° Front Camera left edge + Rear Camera right edge | Dynamic EKF Localizer tracking | **PASS (2 modalities)** |
| **Right** | LiDAR (-120° to -60° sector) | 180° Front Camera right edge + Rear Camera left edge | Dynamic EKF Localizer tracking | **PASS (2 modalities)** |

### Camera 360° Coverage
- **Front Camera:** 180° FOV pointing along +X (`[0, 0, 1, 0]`)
- **Rear Camera:** 180° FOV pointing along -X (`[0, 0, 1, 3.14159]`)
- **Combined FOV:** Complete 360° horizontal coverage without body rotation.
