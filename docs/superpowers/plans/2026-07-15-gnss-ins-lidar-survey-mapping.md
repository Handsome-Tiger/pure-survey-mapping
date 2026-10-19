# GNSS/INS/LiDAR Survey Mapping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and run a pure-survey GNSS/INS/LiDAR pipeline that removes flight-strip ghosting, preserves RTK absolute coordinates, and exports a calibrated RGB point cloud for the complete `HKisland_GNSS02.bag`.

**Architecture:** Extend the current error-state navigation with RTK quality, velocity, and yaw observations; deskew Livox frames with timestamped poses; derive keyframe scan constraints with GNSS-seeded ICP; optimize the keyframe pose graph with g2o while retaining RTK priors; regenerate and voxel-filter the full map; color it using measured LiDAR-camera calibration, lens distortion, corrected image timestamps, and visibility checks.

**Tech Stack:** Python 3, NumPy, SciPy `cKDTree`/`least_squares`, ROS1 `rosbag`, OpenCV, system PCL utilities, system g2o, pytest, ASCII PLY.

## Global Constraints

- Do not call or consume final geometry from `mapping-fusion`.
- Keep map scale fixed and preserve the RTK-derived ENU coordinate frame.
- Use the full bag `/home/shuai/code/datasets/HKisland_GNSS02.bag` after sample validation.
- Default colored PLY fields are exactly `x y z red green blue`.
- Reject bad LiDAR constraints instead of forcing convergence.
- Report internal residuals honestly; do not label them absolute accuracy without control points.
- The workspace is not a Git repository, so each task ends with tests and an artifact checkpoint rather than a Git commit.

---

### Task 1: Parse RTK Auxiliary Measurements

**Files:**
- Modify: `survey_mapping/rosraw.py`
- Modify: `scripts/run_mapping_full_stream.py`
- Test: `tests/test_rosraw.py`

**Interfaces:**
- Produces: `parse_vector3_stamped(data) -> Vector3Stamped`, `parse_int16(data) -> ScalarStamped`, and `parse_uint8(data) -> ScalarStamped`.
- Produces extracted CSV files `rtk_velocity.csv`, `rtk_yaw.csv`, `rtk_quality.csv`, and `livox_imu.csv`.

- [ ] Write binary-message parser tests with known timestamps and values.
- [ ] Run `python3 -m pytest -q tests/test_rosraw.py` and verify the new imports/tests fail.
- [ ] Add focused dataclasses and parsers using the existing `_unpack` and `parse_header` conventions.
- [ ] Extend navigation extraction to retain RTK velocity, yaw, quality, and Livox IMU without changing existing CSV schemas.
- [ ] Run `python3 -m pytest -q tests/test_rosraw.py`; expected result is all tests passing.

### Task 2: Correct Fixed-RTK GNSS/INS Fusion

**Files:**
- Modify: `survey_mapping/error_state_ins.py`
- Modify: `scripts/run_mapping_full_stream.py`
- Modify: `config/HKisland_GNSS02_kfgins_sample.yaml`
- Modify: `config/HKisland_GNSS02_kfgins_all.yaml`
- Test: `tests/test_error_state_ins.py`

**Interfaces:**
- Produces: `update_velocity(velocity_nav, sigma) -> (accepted, residual)`.
- Produces: `update_yaw(yaw_rad, sigma_rad) -> (accepted, residual_rad)`.
- Consumes RTK quality value 50 as fixed solution and selects `fixed_rtk_sigma_m: 0.05`; non-fixed observations use `float_rtk_sigma_m` or are rejected.

- [ ] Add failing tests proving velocity updates reduce velocity error and wrapped yaw updates reduce heading error.
- [ ] Run `python3 -m pytest -q tests/test_error_state_ins.py`; expected failure is missing update methods.
- [ ] Implement 3D velocity and wrapped-yaw Kalman updates with Joseph covariance form.
- [ ] Change external attitude handling so attitude is initialized/observed without overwriting then integrating the same interval twice.
- [ ] Add dynamic RTK sigma selection and residual reporting to `run_filter_error_state`.
- [ ] Run focused tests and the full test suite.
- [ ] Run the extracted full navigation data and require mean fused-to-fixed-RTK position difference below `0.35 m`.

### Task 3: Deskew Livox Scans and Build Keyframes

**Files:**
- Create: `survey_mapping/lidar_odometry.py`
- Create: `scripts/build_lidar_keyframes.py`
- Test: `tests/test_lidar_odometry.py`

**Interfaces:**
- Produces: `deskew_points(points, frame_time, pose_series, reference_time, r_bl, t_bl) -> ndarray[N,4]`.
- Produces: `voxel_centroids(points_xyz, voxel_size) -> ndarray[M,3]`.
- Produces keyframe NPZ files containing `time`, `pose_initial`, `points_local`, and `points_world_initial`.

- [ ] Add a failing synthetic-motion test showing deskew collapses points sampled from one static plane.
- [ ] Run the focused test and verify failure is due to missing deskew implementation.
- [ ] Implement per-point pose sampling and compensation to a common keyframe time.
- [ ] Implement deterministic voxel centroid aggregation and blind/range filtering.
- [ ] Build one keyframe every configured frame/time/distance threshold and persist compact NPZ artifacts.
- [ ] Run tests, then build sample keyframes from 120 LiDAR frames and verify finite coordinates and deterministic counts.

### Task 4: Generate GNSS-Seeded LiDAR Constraints

**Files:**
- Modify: `survey_mapping/lidar_odometry.py`
- Create: `scripts/build_lidar_constraints.py`
- Test: `tests/test_lidar_registration.py`

**Interfaces:**
- Produces: `register_point_to_plane(source, target, initial, max_distance, iterations) -> RegistrationResult`.
- `RegistrationResult` contains `transform`, `rmse`, `inlier_fraction`, `converged`, and `iterations`.
- Produces `lidar_constraints.csv` and `lidar_constraints_rejected.csv`.

- [ ] Add a failing synthetic rigid-transform test with outliers and a GNSS-quality initial guess.
- [ ] Run the test and verify failure due to the missing registration function.
- [ ] Implement nearest-neighbor matching with SciPy `cKDTree`, distance gating, SVD rigid alignment, and convergence checks.
- [ ] Reject constraints when inlier fraction, RMSE, translation correction, rotation correction.
- [ ] Generate adjacent-keyframe constraints and spatially close/time-separated strip constraints.
- [ ] Run sample registration and verify accepted edges reduce pairwise RMSE while no edge exceeds configured correction limits.

### Task 5: Optimize the RTK-Anchored Pose Graph

**Files:**
- Create: `survey_mapping/pose_graph.py`
- Create: `scripts/optimize_lidar_pose_graph.py`
- Test: `tests/test_pose_graph.py`

**Interfaces:**
- Produces: `write_g2o_graph(initial_poses, lidar_edges, rtk_priors, path)`.
- Produces: `run_g2o(input_path, output_path, iterations, robust_kernel) -> subprocess result`.
- Produces: `read_g2o_poses(path) -> list[Pose]`.
- Outputs `trajectory_optimized_keyframes.csv`, `pose_graph.g2o`, `pose_graph_optimized.g2o`, and `optimization_report.json`.

- [ ] Add a failing test for g2o SE3 quaternion serialization and optimized-pose parsing.
- [ ] Run focused tests and verify missing APIs fail.
- [ ] Implement dynamic keyframe vertices, fixed RTK-prior vertices, SE3 LiDAR edges, and translation-strong/rotation-weak RTK prior edges.
- [ ] Invoke `/usr/local/bin/g2o` with bounded iterations and parse optimized vertices.
- [ ] Interpolate optimized keyframe corrections over the high-rate navigation trajectory without changing map scale.
- [ ] Reject optimization if RTK residual, strip RMSE, map extent, or pose jumps worsen beyond configured limits.
- [ ] Verify a synthetic drifted graph is corrected toward fixed priors.

### Task 6: Regenerate and Filter the Optimized Geometry

**Files:**
- Create: `scripts/run_joint_mapping.py`
- Create: `survey_mapping/pointcloud.py`
- Test: `tests/test_pointcloud.py`

**Interfaces:**
- Produces streaming map files `cloud_geometry_optimized_dense.ply` and `cloud_geometry_optimized_review.ply`.
- Produces `joint_mapping_report.json` with point counts, bounds, RTK residuals, LiDAR edge statistics, and rejected-edge reasons.

- [ ] Add failing tests for streaming PLY count/header consistency and deterministic voxel RGB/intensity aggregation.
- [ ] Implement a temporary binary chunk format so full-bag points are not retained in RAM.
- [ ] Reproject every valid LiDAR point with the optimized trajectory.
- [ ] Produce dense and `0.15 m` review clouds; apply deterministic statistical/radius filtering only to the review output.
- [ ] Run the sample pipeline end to end and inspect top/side bounds and local cross sections before full-bag execution.

### Task 7: Apply Measured Camera Calibration and Visibility-Aware Colorization

**Files:**
- Modify: `survey_mapping/camera.py`
- Modify: `scripts/colorize_cloud_from_camera.py`
- Modify: `config/HKisland_GNSS02_kfgins_sample.yaml`
- Modify: `config/HKisland_GNSS02_kfgins_all.yaml`
- Test: `tests/test_camera_projection.py`

**Interfaces:**
- Produces: `project_camera_points(points_camera, intrinsics, distortion, image_shape) -> (uv, depth, valid)`.
- Produces: `visible_projection_mask(uv, depth, image_shape, depth_tolerance) -> ndarray[bool]`.
- Consumes measured `R_cl`, `P_cl`, camera intrinsics/distortion, and `image_time_offset_s: 0.1`.

- [ ] Add failing tests for Brown-Conrady distortion, image timestamp offset selection, bilinear sampling, and z-buffer occlusion.
- [ ] Run focused tests and verify missing distortion/visibility APIs fail.
- [ ] Implement measured pinhole projection with `k1,k2,p1,p2` distortion.
- [ ] Transform optimized world points into the camera pose at corrected exposure time rather than projecting all points in the LiDAR-frame body pose.
- [ ] Add depth-buffer visibility filtering and select the best valid image observation.
- [ ] Write RGB-only PLY plus coverage/occlusion/time-offset statistics.
- [ ] Run sample colorization and verify color follows geometry boundaries in multiple views.

### Task 8: Full-Bag Run and Acceptance

**Files:**
- Modify: `docs/pure_survey_colorization.md`
- Create: `outputs_joint_kfgins_all/run_manifest.json`

**Interfaces:**
- Produces final `outputs_joint_kfgins_all/cloud_colorized_optimized_dense.ply` and `cloud_colorized_optimized_review.ply`.

- [ ] Run `python3 -m pytest -q`; expected result is zero failures.
- [ ] Run sample navigation, keyframe, registration, optimization, geometry, and colorization stages.
- [ ] Compare sample metrics with old baselines: `1.32 m` RTK difference, old strip RMSE, and old color calibration.
- [ ] Run the complete 4349-frame bag without LiDAR frame or point limits.
- [ ] Validate PLY headers, declared/actual point counts, finite bounds, RGB channel variation, and report consistency.
- [ ] Record exact commands, elapsed times, input hashes/configuration, output sizes, and quality metrics in `run_manifest.json`.
- [ ] Update project documentation with outputs, CloudCompare usage, metric interpretation, and remaining control-point limitation.
