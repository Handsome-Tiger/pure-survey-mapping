# Full Livox-Deskewed Survey Color Map Results

## Run Scope

- Bag: `/home/shuai/code/datasets/HKisland_GNSS02.bag`
- Configuration: `config/HKisland_GNSS02_joint_all_livox_deskew_color.yaml`
- Active LiDAR frame or point limits: none
- LiDAR messages with valid points after blind-range filtering: 4,349
- Flight interval represented by the cloud: about 434.9 s
- FAST-LIO / FAST-LIVO2 / mapping-fusion / DJI L1 color transfer: not used
- 3D nearest-neighbor color propagation: not used

The bag contains more LiDAR messages than the 4,349 processed scans. The
remaining messages contain no usable returns after the configured validity and
0.8 m blind-range filters; the pipeline did not stop at a frame limit.

## Navigation and Time Alignment

- Navigation model: GNSS/INS error-state filter with external attitude, RTK
  velocity, RTK yaw, quality-aware GNSS covariance, and IMU bias states
- Time convention: `t_navigation = t_livox + offset_s`
- Measured Livox-to-navigation offset: -0.572 s
- Dual-IMU correlation: 0.929582
- Correlation samples: 22,439
- Fixed offset fallback: not used
- UTC/PPS records: 449
- PPS source: RTK for all 449 records
- Accepted GNSS updates: 2,247 of 2,247
- GNSS innovation mean 3D magnitude: 0.227411 m
- GNSS innovation 3D RMSE: 0.276585 m
- GNSS innovation maximum 3D magnitude: 1.766280 m

## LiDAR Geometry and Pose Graph

- Raw Livox returns in processed scans: 313,128,000
- Filtered geometry points: 74,367,651
- Keyframes: 435 at stride 10
- Constraint candidates: 727
- Accepted constraints: 639
- Rejected constraints: 88
- Accepted adjacent constraints: 431
- Accepted cross-strip constraints: 208
- Mean accepted initial nearest-neighbor RMSE: 0.696540 m
- Mean accepted point-to-plane RMSE: 0.200841 m

The first pose-graph attempt exposed a unit imbalance: translation and compact
quaternion residuals used the same information weight, so meter-scale errors
dominated rotation. The final graph uses independent information values:

- RTK translation prior information: 100
- Attitude prior information: 20,000
- LiDAR rotation-to-translation information scale: 100
- Robust kernel: Huber, width 1.0

Final pose corrections:

- Mean translation correction: 0.097934 m
- Maximum translation correction: 1.274467 m (safety limit 3 m)
- Mean rotation correction: 0.322844 deg
- Maximum rotation correction: 2.613691 deg (safety limit 6 deg)

## Camera Color Projection

Each valid LiDAR return is deskewed to the scan reference time with Livox IMU,
transformed through the optimized world pose, transformed into each selected
camera pose, projected with the released camera/LiDAR calibration and
distortion model, visibility-tested with a z-buffer, and sampled with bilinear
interpolation. At most three camera frames inside a +/-0.12 s window are used;
the nearest valid visible observation wins.

- LiDAR frames paired to camera: 4,349 of 4,349
- Decoded camera frames: 4,494
- Mean nearest-camera time difference: 0.0004269 s
- Maximum nearest-camera time difference: 0.0123277 s
- Complete point count: 74,367,651
- Strict true-color point count: 73,125,606
- True-color fraction: 0.983299
- Unobserved complete-cloud points: 1,242,045

Unobserved points remain in the complete PLY as intensity grayscale with
`color_valid = 0`. They are absent from the strict PLY. A single JPEG decoder
warning about a premature data-segment end was emitted, but decoding continued
and no LiDAR frame lost camera pairing.

## Independent PLY Verification

- Complete declared/actual/finite records:
  74,367,651 / 74,367,651 / 74,367,651
- Strict declared/actual/finite records:
  73,125,606 / 73,125,606 / 73,125,606
- Complete true-color count equals strict point count: yes
- Complete bounds minimum XYZ:
  [-349.708679, -579.766479, -351.223999] m
- Complete bounds maximum XYZ:
  [135.255005, 174.413391, 84.202904] m
- Complete color PLY size: 1,189,882,637 bytes
- Strict color PLY size: 1,096,884,284 bytes
- Intensity geometry PLY size: about 922 MiB

## Deliverables

- Complete RGB plus `color_valid`:
  `/home/shuai/cehui/outputs_joint_kfgins_all_livox_deskew_color/cloud_optimized_color_complete.ply`
- Strict camera-observed RGB:
  `/home/shuai/cehui/outputs_joint_kfgins_all_livox_deskew_color/cloud_optimized_color_strict.ply`
- Intensity geometry:
  `/home/shuai/cehui/outputs_joint_kfgins_all_livox_deskew_color/cloud_optimized_geometry_livox_deskew.ply`
- Color report:
  `/home/shuai/cehui/outputs_joint_kfgins_all_livox_deskew_color/full_color_report.json`
- Independent verification report:
  `/home/shuai/cehui/outputs_joint_kfgins_all_livox_deskew_color/color_cloud_verification.json`
- Optimization report:
  `/home/shuai/cehui/outputs_joint_kfgins_all_livox_deskew_color/optimization_report.json`
- Hardware synchronization report:
  `/home/shuai/cehui/outputs_joint_kfgins_all_livox_deskew_color/hardware_time_sync_report.json`

## Accuracy Interpretation

The 0.276585 m GNSS innovation RMSE and 0.200841 m LiDAR point-to-plane RMSE
measure internal consistency. They do not constitute an independent absolute
survey accuracy certificate. Absolute horizontal/vertical accuracy must be
evaluated against surveyed checkpoints or a registered reference point cloud
that was not used by the filter or pose graph.

All configured point-count, timing, color-coverage, finite-coordinate, and
pose-correction safety gates passed.

