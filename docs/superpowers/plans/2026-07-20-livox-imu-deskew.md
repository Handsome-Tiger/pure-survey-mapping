# Livox IMU Time Alignment and Deskew Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add measured Livox-to-navigation time alignment and Livox-IMU-based per-point scan deskew to the pure survey mapping pipeline.

**Architecture:** A standalone time-alignment module estimates the scalar clock offset from dual-IMU angular-speed correlation. A standalone IMU deskew module integrates Livox gyroscopes and converts each return to a common scan reference while retaining the existing GNSS/INS trajectory for translation and global anchoring. Configuration selects the new path, while the old navigation-only path remains compatible.

**Tech Stack:** Python 3, NumPy, SciPy Rotation, pytest, ROS1 rosbag raw-message reader, YAML.

## Global Constraints

- Do not call or import FAST-LIO, FAST-LIVO2, or mapping-fusion.
- Preserve the current outputs and fixed-offset fallback.
- Use `/livox/imu` only for relative scan orientation in this stage.
- Use test-first red-green-refactor for every new behavior.
- New sample outputs go to `outputs_joint_kfgins_sample_livox_deskew`.

---

### Task 1: Dual-IMU Time Offset Estimator

**Files:**
- Create: `survey_mapping/time_alignment.py`
- Create: `tests/test_time_alignment.py`

**Interfaces:**
- Produces: `TimeOffsetEstimate(offset_s, correlation, margin, sample_count)`.
- Produces: `estimate_gyro_time_offset(reference_times, reference_gyro, sensor_times, sensor_gyro, search_min_s, search_max_s, step_s, sample_period_s) -> TimeOffsetEstimate`.
- Offset convention: `t_reference = t_sensor + offset_s`.

- [ ] **Step 1: Write a failing synthetic shifted-signal test**

```python
estimate = estimate_gyro_time_offset(ref_t, ref_w, sensor_t, sensor_w, -0.8, 0.0, 0.01, 0.01)
assert abs(estimate.offset_s + 0.4) <= 0.01
assert estimate.correlation > 0.99
```

- [ ] **Step 2: Run the focused test and verify the module is missing**

Run: `pytest -q tests/test_time_alignment.py`

Expected: collection fails with `ModuleNotFoundError: survey_mapping.time_alignment`.

- [ ] **Step 3: Implement bounded normalized correlation**

Implement validation, common-grid interpolation, gyroscope magnitudes,
per-candidate normalization, peak selection, and peak-margin reporting. Reject
fewer than 100 samples and near-constant angular-speed signals.

- [ ] **Step 4: Run the focused tests**

Run: `pytest -q tests/test_time_alignment.py`

Expected: all time-alignment tests pass.

### Task 2: Livox Gyroscope Rotation Series

**Files:**
- Create: `survey_mapping/imu_deskew.py`
- Create: `tests/test_imu_deskew.py`

**Interfaces:**
- Produces: `ImuRotationSeries(times, quaternions)`.
- Produces: `ImuRotationSeries.from_gyro(times, gyro, bias=None)`.
- Produces: `relative_rotations(sample_times, reference_time) -> scipy Rotation`.

- [ ] **Step 1: Write a failing constant-rate rotation test**

```python
series = ImuRotationSeries.from_gyro(times, gyro)
relative = series.relative_rotations(np.array([0.0]), 1.0)
assert np.allclose(relative.apply([[10.0, 0.0, 0.0]]), [[0.0, -10.0, 0.0]], atol=1e-6)
```

- [ ] **Step 2: Verify the focused test fails because the API is missing**

Run: `pytest -q tests/test_imu_deskew.py`

- [ ] **Step 3: Implement midpoint quaternion integration and interpolation**

Normalize every accumulated quaternion, reject non-increasing timestamps, and
clip interpolation requests to the recorded IMU interval.

- [ ] **Step 4: Run the focused tests**

Run: `pytest -q tests/test_imu_deskew.py`

Expected: constant-rate and validation tests pass.

### Task 3: Per-Point Livox IMU Deskew

**Files:**
- Modify: `survey_mapping/imu_deskew.py`
- Modify: `tests/test_imu_deskew.py`

**Interfaces:**
- Produces: `deskew_points_with_imu(points, frame_time, reference_time, imu_rotations, pose_series, navigation_time_offset_s, rotation_body_lidar, translation_body_lidar, rotation_imu_lidar, translation_imu_lidar) -> ndarray[N,4]`.

- [ ] **Step 1: Add failing rotating-scan and translating-scan tests**

Use two observations of one static point and require both outputs to coincide in
the end-of-scan LiDAR frame.

- [ ] **Step 2: Run the tests and verify geometric assertions fail**

Run: `pytest -q tests/test_imu_deskew.py`

- [ ] **Step 3: Implement vectorized relative rotation and navigation translation**

Use the Livox-IMU relative orientation for point vectors and the GNSS/INS pose
series only for scan-internal origin displacement. Apply both extrinsics using
the documented LiDAR-to-IMU convention.

- [ ] **Step 4: Run the focused tests**

Run: `pytest -q tests/test_imu_deskew.py`

Expected: all synthetic static points collapse at the reference time.

### Task 4: Pipeline and Configuration Integration

**Files:**
- Modify: `scripts/build_lidar_keyframes.py`
- Modify: `scripts/build_optimized_cloud.py`
- Modify: `scripts/run_mapping_full_stream.py`
- Create: `config/HKisland_GNSS02_joint_sample_livox_deskew.yaml`
- Create: `tests/test_livox_deskew_config.py`

**Interfaces:**
- Produces: `resolve_time_alignment(config, dji_imu, livox_imu) -> TimeOffsetEstimate`.
- Configuration keys: `time_alignment.method`, search bounds, grid step,
  sample period, minimum correlation, and fixed fallback.
- Configuration key: `mapping.deskew_orientation_source` with values
  `navigation` or `livox_imu`.

- [ ] **Step 1: Write failing configuration-resolution tests**

Verify automatic mode returns a measured offset and fixed mode returns the
configured fallback without estimating.

- [ ] **Step 2: Run focused tests and verify failure**

Run: `pytest -q tests/test_livox_deskew_config.py`

- [ ] **Step 3: Integrate the new modules and write the report**

Load `livox_imu.csv`, estimate once per output root, save
`time_alignment_report.json`, select deskew mode in keyframe generation, and use
the same mode during optimized cloud regeneration.

- [ ] **Step 4: Run focused and full tests**

Run: `pytest -q tests/test_time_alignment.py tests/test_imu_deskew.py tests/test_livox_deskew_config.py`

Run: `pytest -q`

Expected: all tests pass.

### Task 5: Sample-Bag Verification

**Files:**
- Create: `outputs_joint_kfgins_sample_livox_deskew/time_alignment_report.json`
- Create: `outputs_joint_kfgins_sample_livox_deskew/keyframe_report.json`
- Create: `outputs_joint_kfgins_sample_livox_deskew/optimized_cloud_report.json`
- Create: `docs/2026-07-20-livox-imu-deskew-results.md`

**Interfaces:**
- Consumes the stage-one sample configuration and current ROS bag.
- Produces measured timing, point-count, and registration comparisons.

- [ ] **Step 1: Build sample keyframes**

Run: `python3 scripts/build_lidar_keyframes.py --config config/HKisland_GNSS02_joint_sample_livox_deskew.yaml`

Expected: 24 keyframes from 120 selected scans and a correlation above `0.80`.

- [ ] **Step 2: Build constraints and optimize the sample graph**

Run: `python3 scripts/build_lidar_constraints.py --config config/HKisland_GNSS02_joint_sample_livox_deskew.yaml`

Run: `python3 scripts/optimize_lidar_pose_graph.py --config config/HKisland_GNSS02_joint_sample_livox_deskew.yaml`

- [ ] **Step 3: Regenerate the sample geometry**

Run: `python3 scripts/build_optimized_cloud.py --config config/HKisland_GNSS02_joint_sample_livox_deskew.yaml`

Expected: finite binary PLY, declared vertex count equals the written count.

- [ ] **Step 4: Record measured comparison**

Document the estimated offset, correlation, accepted/rejected edge counts,
registration RMSE, cloud point count, and any remaining limitations. Do not
claim improved accuracy unless the measured comparison supports it.
