# LiDAR/Navigation Time Alignment Correction

## Symptom

The previous full map showed repeated outlines, curved ghost layers, and an unreadable building on the southeast island. This was a geometry problem, not a CloudCompare color-display problem.

## Root Cause

The mapping code sampled DJI navigation at the raw Livox timestamp. Two independent diagnostics showed that the navigation state must instead be sampled about 0.60 seconds earlier:

- DJI and Livox IMU angular-speed correlation peaks at `-0.575 s` with correlation `0.9232`.
- For 380 high-quality adjacent LiDAR constraints, the median rotation disagreement falls from `0.670 deg` at zero offset to `0.214 deg` at `-0.60 s`.

The configured LiDAR-to-body rotation was also checked. Synchronized IMU vector alignment differs from the configured rotation by only about `0.65 deg`, and gravity-direction cosine similarity is `0.9990`, so the dominant error was timing rather than mounting-axis selection.

## Fix

`mapping.navigation_time_offset_s: -0.60` is now applied consistently to:

- per-point Livox deskew;
- initial keyframe poses;
- full geometry reprojection;
- camera-exposure navigation poses used during colorization.

The camera/LiDAR timestamp correction remains separately configured as `camera.time_offset_s: 0.1`.

## Full-Bag Comparison

| Metric | Previous | Time-aligned |
| --- | ---: | ---: |
| Accepted LiDAR constraints | 557 | 652 |
| Rejected constraints | 199 | 102 |
| Graph components | 38 | 2 |
| Mean initial NN RMSE | 1.012 m | 0.846 m |
| Mean point-to-plane RMSE | 0.259 m | 0.242 m |
| Mean pose translation correction | 0.103 m | 0.092 m |
| Maximum pose translation correction | 1.705 m | 1.287 m |

## Recommended Outputs

- `outputs_joint_kfgins_all_timealigned/cloud_optimized_colorized_clean.ply`
- `outputs_joint_kfgins_all_timealigned/cloud_optimized_geometry_clean.ply`
- `outputs_joint_kfgins_all_timealigned/cloud_optimized_colorized_clean_preview.png`
- `outputs_joint_kfgins_all_timealigned/time_alignment_report.json`

The old results remain under `outputs_joint_kfgins_all` for direct comparison.
