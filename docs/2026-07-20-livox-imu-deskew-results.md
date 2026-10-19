# Livox IMU Deskew Stage-One Results

## Scope

This stage keeps the project as a pure survey-mapping implementation. It does
not call FAST-LIO, FAST-LIVO2, or mapping-fusion. The existing DJI GNSS/INS
trajectory remains the global anchor; `/livox/imu` supplies only the relative
orientation within each LiDAR scan.

## Implemented Changes

- Added dual-IMU angular-speed correlation with the convention
  `t_navigation = t_livox + offset_s`.
- Added hardware synchronization diagnostics for DJI GPS UTC, FC UTC, and PPS
  source messages.
- Added midpoint Livox gyroscope integration and quaternion interpolation.
- Added per-return rotation and translation compensation to the end-of-scan
  LiDAR frame.
- Applied the official MARS-LVIG LiDAR-to-IMU extrinsic:
  - rotation: identity
  - translation: `[0.04165, 0.02326, -0.0284] m`
- Integrated the same deskew method into both keyframe construction and final
  optimized-cloud regeneration.
- Preserved `mapping.deskew_orientation_source: navigation` and fixed time
  offset mode as backward-compatible fallbacks.

## Time Alignment Evidence

The full extracted IMU streams contain 179,719 DJI IMU samples and 93,692
Livox IMU samples. Livox timestamps are strictly increasing, with median sample
interval about `4.975 ms`.

Dual-IMU correlation produced:

- offset: `-0.572 s`
- correlation: `0.929582`
- evaluated samples: `22,439`

Hardware synchronization topics produced:

- GPS UTC messages: `449`
- FC UTC messages: `449`
- PPS source messages: `449`
- PPS source: `RTK` for all `449` records
- FC-message minus declared-UTC median: `-0.562424 s`

The FC UTC result and dual-IMU result differ by about `9.6 ms`. This independent
agreement supports the measured offset and explains why the earlier empirical
`-0.60 s` correction had the correct sign and approximate magnitude.

## Sample Run

Configuration:

`config/HKisland_GNSS02_joint_sample_livox_deskew.yaml`

The sample starts at LiDAR frame 500, processes 120 scans, and selects one
keyframe every five scans.

- selected scans: `120`
- keyframes: `24`
- accepted LiDAR edges: `23 / 23`
- mean initial nearest-neighbor RMSE: `0.593789 m`
- mean point-to-plane RMSE: `0.199412 m`
- optimized mean translation correction: `0.004862 m`
- optimized mean rotation correction: `1.202502 deg`
- exported points: `2,727,590`
- PLY size: `35,458,829 bytes`
- PLY declared/actual records: `2,727,590 / 2,727,590`
- finite coordinates: all points
- coordinate bounds: `[-89.405, -84.850, -9.219]` to
  `[58.050, 63.795, 14.064] m`

Output:

`outputs_joint_kfgins_sample_livox_deskew/cloud_optimized_geometry_livox_deskew.ply`

## Baseline Comparison

The previous navigation-deskew sample used the same 120 scans and 24
keyframes:

| Metric | Previous | Livox IMU | Change |
|---|---:|---:|---:|
| Initial nearest-neighbor RMSE | 0.596043 m | 0.593789 m | -0.002254 m |
| Point-to-plane RMSE | 0.199781 m | 0.199412 m | -0.000369 m |
| Accepted edges | 23 | 23 | unchanged |

The improvement is real but small on this short segment. This stage removes a
scan-internal timing/orientation inconsistency; it does not yet correct the
larger trajectory and global attitude errors that cause duplicated building
surfaces over the complete flight.

## Next Bottleneck

The next stage should replace the current external-attitude overwrite behavior
with a self-written Livox-IMU error-state propagation. DJI RTK position and
velocity should update that filter, while DJI attitude becomes an initialization
or weak observation instead of a hard per-sample assignment. This keeps the
project independent while addressing inter-scan trajectory errors that deskew
alone cannot remove.
