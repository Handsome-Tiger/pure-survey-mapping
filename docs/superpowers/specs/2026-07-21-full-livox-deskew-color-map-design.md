# Full Livox-Deskewed Color Map Design

## Goal

Generate a complete, camera-colored point cloud from every valid LiDAR scan in
`HKisland_GNSS02.bag` using only the pure survey-mapping project. FAST-LIO,
FAST-LIVO2, mapping-fusion, DJI L1 color transfer, and spatial nearest-neighbor
color propagation are excluded.

## Deliverables

The full run uses output root:

`outputs_joint_kfgins_all_livox_deskew_color`

It produces:

- `cloud_optimized_color_complete.ply`: every accepted geometry point. Points
  with a valid camera observation store true RGB; unobserved points store
  intensity grayscale. A `color_valid` scalar distinguishes the two cases.
- `cloud_optimized_color_strict.ply`: only points that pass camera time,
  projection, image-bound, positive-depth, and visibility checks.
- `cloud_optimized_geometry_livox_deskew.ply`: the complete intensity geometry
  used by both color products.
- `full_color_report.json`: frame, point, timing, color-coverage, coordinate
  bounds, and file-integrity statistics.
- Existing navigation, keyframe, LiDAR-constraint, optimization, and hardware
  time-synchronization reports in the same output root.

The implementation first produces equivalent 120-frame sample outputs under
`outputs_joint_kfgins_sample_livox_deskew_color`. The full bag is processed only
after the sample passes the acceptance gates below.

## Full Geometry Pipeline

1. Reuse only extracted observation CSV files from a previous run when they
   are from the same bag. Do not reuse keyframes, constraints, optimized poses,
   geometry, or colors.
2. Estimate the Livox-to-DJI offset from full dual-IMU gyroscope correlation
   with the convention `t_navigation = t_livox + offset_s`.
3. Verify the result against FC UTC and RTK PPS diagnostics. Reject correlation
   below `0.80`; do not silently use the fixed fallback in the production
   configuration.
4. Remove `max_lidar_frames` and `max_points_per_frame` from the full
   configuration. Process every valid scan after `start_lidar_frame: 0`.
5. Integrate `/livox/imu` gyroscopes and deskew every return to the end-of-scan
   LiDAR frame using the official LiDAR-to-IMU extrinsic.
6. Anchor each scan with the existing GNSS/INS pose, construct LiDAR keyframes,
   estimate point-to-plane constraints, and optimize the RTK-anchored pose
   graph.
7. Rebuild all geometry from the raw Livox returns with the same deskew and
   optimized correction used during keyframe registration.

## Camera Time and Pose Model

The measured camera-to-Livox header correction remains
`camera.time_offset_s: +0.1`, so:

```text
t_livox_camera = t_camera_header + 0.1
t_navigation_camera = t_livox_camera + dt_livox_to_navigation
```

For each camera candidate, the initial camera body pose is sampled from the
navigation trajectory at `t_navigation_camera`. The pose-graph correction at
`t_livox_camera` is left-composed with that initial pose. This keeps optimized
LiDAR points and optimized camera poses in the same world frame, including for
adjacent-frame color candidates.

## Per-Point Color Data Flow

For each LiDAR frame:

1. Parse and filter Livox returns with `blind_m: 0.8` and existing tag/return
   validity rules.
2. Deskew all points to the scan reference time with Livox IMU relative
   orientation and navigation translation.
3. Transform the reference-frame points into the initial world frame, then
   apply the interpolated optimized correction.
4. Select up to three camera candidates around the scan reference time within
   `camera.multi_view_window_s: 0.12`.
5. Transform each optimized world point into each candidate camera using the
   corrected camera pose and measured body-LiDAR-camera extrinsics.
6. Apply the released camera intrinsics and distortion coefficients.
7. Reject negative depth, out-of-image pixels, and occluded points using the
   existing pixel z-buffer with `depth_tolerance_m: 0.3`.
8. Rank valid observations by absolute time difference, then distance from the
   image center. Use one best observation per point; do not average conflicting
   colors across frames.
9. Sample BGR pixels with bilinear interpolation and write RGB values.
10. Write every point to the complete cloud. Write only truly colored points
    to the strict cloud.

This method does not fill missing colors from nearby 3D points. Gray points in
the complete cloud therefore mean no reliable camera observation, not a failed
RGB import.

## Streaming and File Format

The 11.8 GB bag and full point cloud must be processed frame by frame. At most
one LiDAR frame, three decoded camera images, and their projection arrays are
held in memory.

Both PLY files use binary little-endian records:

```text
complete: x y z red green blue color_valid
strict:   x y z red green blue
```

The writer patches the declared vertex count on successful close. Interrupted
runs do not get reported as valid deliverables.

## Failure Handling

- Missing extracted IMU observations, camera frames, keyframe poses, or
  optimized corrections stop the run with a descriptive error.
- Unsupported deskew or alignment modes stop before opening output files.
- Time-alignment correlation below `0.80` stops the production run.
- A sample with zero paired frames, zero true-color points, non-finite
  coordinates, or mismatched PLY counts blocks the full run.
- Existing output files live in separate sample/full directories so the
  previous validated maps are not overwritten.

## Sample Acceptance Gates

- Process exactly 120 selected LiDAR scans and 24 keyframes.
- Complete-cloud point count equals the geometry point count.
- Strict-cloud point count equals the number of `color_valid == 1` records.
- All coordinates are finite and all RGB values are unsigned bytes.
- At least 95% of selected LiDAR frames have an eligible synchronized camera
  frame.
- Mean nearest-camera time difference is at most `0.03 s`.
- True-color coverage is reported, not hidden. A value below `10%` blocks the
  full run and triggers calibration/time review.
- Existing unit and regression tests pass.

## Full-Run Acceptance Gates

- `max_lidar_frames`, `max_points_per_frame`, and frame-start clipping are not
  active.
- Process all valid LiDAR scans from the bag, expected near 4,493 frames.
- PLY declared counts equal actual binary records for geometry, complete
  color, and strict color outputs.
- Complete-cloud count equals the accepted full geometry count.
- All output coordinates are finite.
- Report true-color fraction, paired-frame fraction, mean/max camera time
  difference, bounds, file sizes, and time-alignment evidence.
- Do not claim absolute map accuracy from GNSS residuals alone. DJI L1 map
  comparison remains a later independent accuracy-validation stage.

## Compatibility

- Preserve `mapping.deskew_orientation_source: navigation` and fixed time
  alignment as explicit legacy modes.
- Preserve existing intensity-only and strict-visible output behavior for old
  configurations unless the new complete/strict dual-output mode is enabled.
- The project remains runnable without any FAST-LIO or FAST-LIVO installation.
