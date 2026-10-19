# Livox IMU Time Alignment and Deskew Design

## Goal

Improve the pure survey-mapping pipeline without FAST-LIO, FAST-LIVO2, or
mapping-fusion by using the bag's `/livox/imu` measurements for scan-internal
rotation compensation and by estimating the Livox-to-DJI clock offset from the
two recorded IMU streams.

## Observed Data Facts

- `/livox/lidar` runs at about 10 Hz and stores a nanosecond offset for every
  return.
- `/livox/imu` runs at about 200 Hz and uses the same sensor time domain as the
  Livox point cloud.
- `/dji_osdk_ros/imu` runs at about 400 Hz and uses the DJI/ROS navigation time
  domain.
- Gyroscope-magnitude correlation over the full extracted streams gives a
  Livox-to-navigation offset near `-0.570 s` with correlation near `0.929`.
  The existing manually selected `-0.60 s` therefore has the correct sign but
  should be a measured result, not an unexplained constant.
- The official MARS-LVIG Livox-IMU extrinsic uses identity rotation and
  translation `[0.04165, 0.02326, -0.0284] m` from LiDAR to IMU coordinates.

## Architecture

### Clock alignment

Estimate a scalar offset `dt_livox_to_nav` satisfying:

```text
t_navigation = t_livox + dt_livox_to_nav
```

The estimator resamples both gyroscope magnitudes, evaluates normalized
correlation over a bounded offset grid, and reports the peak score and the
score margin. Stationary or insufficient data is rejected. The result is
written to `time_alignment_report.json` and is reused by downstream mapping
steps. A configured fixed offset remains available as a fallback.

The DJI PPS/UTC topics are diagnostic evidence for the acquisition clock and
will be recorded in the report, but they do not directly reveal the remaining
Livox driver timestamp shift. Dynamic IMU correlation therefore supplies the
cross-sensor offset used by mapping.

### Livox IMU orientation interpolation

Integrate bias-corrected Livox angular velocity with midpoint integration to
obtain a quaternion series in the Livox IMU time domain. Only relative
orientation within a 0.1 s LiDAR scan is used, so long-duration heading drift
does not become a global attitude source in this stage.

### Per-point scan deskew

For every LiDAR return:

1. Compute `t_point = t_frame + offset_time_ns * 1e-9`.
2. Interpolate Livox IMU rotations at `t_point` and at the scan reference time.
3. Rotate each point from its acquisition orientation into the reference
   orientation.
4. Interpolate the existing navigation positions at
   `t_point + dt_livox_to_nav` to compensate scan-internal translation.
5. Apply the LiDAR/IMU and body/LiDAR lever arms consistently.

The global reference pose still comes from the current GNSS/INS trajectory.
This stage changes scan-internal compensation only; it does not introduce a
FAST-LIO-style odometry backend.

## Integration

- Keyframe generation selects `mapping.deskew_orientation_source`.
- `livox_imu` uses the new implementation.
- `navigation` preserves the existing implementation.
- Optimized dense-cloud regeneration uses the same deskew selection so the
  registration keyframes and final point cloud cannot disagree.
- A new configuration and output directory keep the current time-aligned map
  unchanged for direct comparison.

## Acceptance Criteria

- Synthetic offset recovery error is at most one configured grid step.
- Synthetic rotating and translating scans collapse to one reference-frame
  point within numerical tolerance.
- Existing tests remain green.
- The sample bag run reports a finite offset with correlation at least `0.80`.
- The sample keyframe and optimized-cloud pipelines complete without NaN or
  count mismatches.
- Registration RMSE and accepted-edge count are compared with the existing
  sample baseline; no claim of accuracy improvement is made unless measured.

## Non-Goals

- No FAST-LIO or FAST-LIVO2 executable, library, source code, or output is used.
- No LiDAR scan-to-map update is added to the navigation filter in this stage.
- No full-bag color rerun is required until sample geometry passes inspection.
