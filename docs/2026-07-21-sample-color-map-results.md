# Sample Livox-Deskewed Color Map Results

## Scope

- Bag: `/home/shuai/code/datasets/HKisland_GNSS02.bag`
- Configuration: `config/HKisland_GNSS02_joint_sample_livox_deskew_color.yaml`
- LiDAR frames: 500 through 619 (120 frames)
- Per-frame input cap: 72,000 raw Livox returns before validity filtering
- Keyframe stride: 5
- FAST-LIO / FAST-LIVO2 / mapping-fusion: not used

## Geometry and Optimization

- Selected LiDAR frames: 120
- Keyframes: 24
- Accepted LiDAR constraints: 23 of 23
- Mean initial nearest-neighbor RMSE: 0.593789 m
- Mean accepted point-to-plane RMSE: 0.199412 m
- Mean translation correction: 0.004862 m
- Maximum translation correction: 0.008816 m
- Mean rotation correction: 1.202502 deg
- Maximum rotation correction: 1.919817 deg
- Geometry point count: 2,727,590

## Time Alignment

- Convention: `t_navigation = t_livox + offset_s`
- Measured offset: -0.572 s
- Dual-IMU correlation: 0.929582
- Fixed fallback: not used

## Color Projection

- Complete point count: 2,727,590
- Strict true-color point count: 2,654,423
- True-color fraction: 0.973175
- Paired LiDAR frames: 120 of 120
- Paired-frame fraction: 1.0
- Mean nearest-camera time difference: 0.0001725 s
- Maximum nearest-camera time difference: 0.0008099 s
- Multi-view window: +/-0.12 s
- Maximum camera candidates per LiDAR frame: 3

## Independent PLY Verification

- Complete declared/actual/finite: 2,727,590 / 2,727,590 / 2,727,590
- Strict declared/actual/finite: 2,654,423 / 2,654,423 / 2,654,423
- Complete bounds min: [-89.404610, -84.850327, -9.219361] m
- Complete bounds max: [58.049595, 63.794910, 14.063907] m
- Complete file size: 43,641,661 bytes
- Strict file size: 39,816,539 bytes
- Complete/strict true-color counts consistent: yes

## Outputs

- Geometry: `/home/shuai/cehui/outputs_joint_kfgins_sample_livox_deskew_color/cloud_optimized_geometry_livox_deskew.ply`
- Complete color: `/home/shuai/cehui/outputs_joint_kfgins_sample_livox_deskew_color/cloud_optimized_color_complete.ply`
- Strict color: `/home/shuai/cehui/outputs_joint_kfgins_sample_livox_deskew_color/cloud_optimized_color_strict.ply`
- Color report: `/home/shuai/cehui/outputs_joint_kfgins_sample_livox_deskew_color/full_color_report.json`
- Verification report: `/home/shuai/cehui/outputs_joint_kfgins_sample_livox_deskew_color/color_cloud_verification.json`

All configured sample acceptance gates passed.

