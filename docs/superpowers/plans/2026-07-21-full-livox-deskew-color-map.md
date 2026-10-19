# Full Livox-Deskewed Color Map Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rebuild every valid scan in `HKisland_GNSS02.bag` with Livox-IMU deskew and export both complete-geometry RGB and strict true-color point clouds from measured camera projection.

**Architecture:** The existing GNSS/INS and LiDAR pose-graph pipeline remains the geometry authority. New focused modules add corrected camera-pose sampling, a bounded multi-view camera window, conservative projection-based color selection, and a dual binary PLY writer. The sample run must pass timing, coverage, count, and finite-coordinate gates before the same configuration is expanded to the complete bag.

**Tech Stack:** Python 3.10, NumPy, SciPy Rotation, OpenCV image decoding, ROS1 rosbag, pytest, binary little-endian PLY.

## Global Constraints

- Do not call or import FAST-LIO, FAST-LIVO2, mapping-fusion, or DJI L1 color transfer.
- Do not propagate color through 3D nearest neighbors.
- Use `t_navigation = t_livox + offset_s` and require dual-IMU correlation at least `0.80`.
- Keep `camera.time_offset_s: +0.1` and the released camera/LiDAR calibration values unchanged.
- The full configuration must not contain active `start_lidar_frame`, `max_lidar_frames`, or `max_points_per_frame` limits.
- Unobserved points remain in the complete cloud as intensity grayscale with `color_valid = 0`.
- Only camera-projected, z-buffer-visible points appear in the strict cloud.
- Process data frame by frame; do not retain the full cloud or all decoded images in RAM.
- Existing output directories and validated PLY files must not be overwritten.
- The current workspace is not a Git repository, so commit steps are recorded as unavailable rather than fabricating commit history.

---

## File Map

- Create `survey_mapping/color_output.py`: dual complete/strict binary PLY writer.
- Create `survey_mapping/color_fusion.py`: corrected camera views, bounded camera window, and conservative multi-view projection.
- Modify `survey_mapping/optimized_mapping.py`: apply pose-graph corrections to body/camera poses as well as points.
- Modify `scripts/build_optimized_color_cloud.py`: Livox deskew, optimized camera poses, streaming dual output, and report generation.
- Create `scripts/verify_color_cloud.py`: independent binary-record, finite-coordinate, RGB, validity, and count verification.
- Create sample and full YAML configurations with separate output roots.
- Add focused tests for each new public interface.

### Task 1: Dual Complete/Strict Binary PLY Writer

**Files:**
- Create: `survey_mapping/color_output.py`
- Create: `tests/test_color_output.py`

**Interfaces:**
- Produces: `DualColorPlyWriter(complete_path, strict_path)` context manager.
- Produces: `write(points_xyz, rgb, color_valid, intensity) -> None`.
- Complete record: `xyz float32[3]`, `rgb uint8[3]`, `color_valid uint8`.
- Strict record: `xyz float32[3]`, `rgb uint8[3]` for `color_valid == 1` only.
- Exposes: `complete_count`, `strict_count`, `true_color_count`.

- [ ] **Step 1: Write failing dual-output tests**

```python
import numpy as np

from survey_mapping.color_output import DualColorPlyWriter, read_ply_vertex_count


def test_dual_writer_preserves_all_points_and_filters_strict_points(tmp_path):
    complete = tmp_path / "complete.ply"
    strict = tmp_path / "strict.ply"
    xyz = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]], dtype=float)
    rgb = np.array([[10, 20, 30], [40, 50, 60], [70, 80, 90]], dtype=np.uint8)
    valid = np.array([True, False, True])
    intensity = np.array([1, 120, 3], dtype=np.uint8)

    with DualColorPlyWriter(complete, strict) as writer:
        writer.write(xyz, rgb, valid, intensity)

    assert read_ply_vertex_count(complete) == 3
    assert read_ply_vertex_count(strict) == 2
    complete_bytes = complete.read_bytes()
    assert b"property uchar color_valid" in complete_bytes[:512]
```

- [ ] **Step 2: Run the focused test and verify RED**

Run: `python3 -m pytest -q tests/test_color_output.py`

Expected: collection fails because `survey_mapping.color_output` does not exist.

- [ ] **Step 3: Implement the dual writer**

```python
class DualColorPlyWriter:
    def write(self, points_xyz, rgb, color_valid, intensity):
        xyz = np.asarray(points_xyz, dtype=np.float32)
        colors = np.asarray(rgb, dtype=np.uint8).copy()
        valid = np.asarray(color_valid, dtype=bool)
        gray = np.asarray(intensity, dtype=np.uint8).reshape(-1)
        colors[~valid] = gray[~valid, None]
        complete = np.empty(len(xyz), dtype=self.COMPLETE_DTYPE)
        complete["xyz"] = xyz
        complete["rgb"] = colors
        complete["color_valid"] = valid.astype(np.uint8)
        self.complete_handle.write(complete.tobytes())
        strict = np.empty(np.count_nonzero(valid), dtype=self.STRICT_DTYPE)
        strict["xyz"] = xyz[valid]
        strict["rgb"] = colors[valid]
        self.strict_handle.write(strict.tobytes())
        self.complete_count += len(xyz)
        self.strict_count += len(strict)
        self.true_color_count += int(np.count_nonzero(valid))
```

Implement fixed-width count patching on successful context exit. Validate equal
array lengths, finite coordinates, and `rgb.shape == (N, 3)` with descriptive
`ValueError` messages.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run: `python3 -m pytest -q tests/test_color_output.py`

Expected: all writer tests pass.

- [ ] **Step 5: Record checkpoint**

Record changed files and test output in the task log. Git commit is unavailable
because `/home/shuai/cehui` is not a Git repository.

### Task 2: Correct Camera Poses with the Optimized Trajectory

**Files:**
- Modify: `survey_mapping/optimized_mapping.py`
- Create: `tests/test_optimized_pose_correction.py`

**Interfaces:**
- Produces: `TransformCorrectionInterpolator.matrices_at(sample_times) -> ndarray[N,4,4]`.
- Produces: `TransformCorrectionInterpolator.correct_poses(positions, quaternions, sample_times) -> tuple[ndarray[N,3], ndarray[N,4]]`.
- Correction convention: `T_world_body_optimized = C_world(sample_time) @ T_world_body_initial`.

- [ ] **Step 1: Write a failing left-composition pose test**

```python
def test_correct_poses_left_composes_world_correction():
    correction = np.eye(4)
    correction[:3, 3] = [10.0, 0.0, 0.0]
    interpolator = TransformCorrectionInterpolator([0.0, 1.0], [correction, correction])
    position, quaternion = interpolator.correct_poses(
        np.array([[1.0, 2.0, 3.0]]),
        np.array([[0.0, 0.0, 0.0, 1.0]]),
        np.array([0.5]),
    )
    assert np.allclose(position, [[11.0, 2.0, 3.0]])
    assert np.allclose(quaternion, [[0.0, 0.0, 0.0, 1.0]])
```

- [ ] **Step 2: Run the test and verify RED**

Run: `python3 -m pytest -q tests/test_optimized_pose_correction.py`

Expected: `AttributeError` for `correct_poses`.

- [ ] **Step 3: Implement correction matrices and pose composition**

```python
def correct_poses(self, positions, quaternions, sample_times):
    corrections = self.matrices_at(sample_times)
    initial = np.repeat(np.eye(4)[None, :, :], len(corrections), axis=0)
    initial[:, :3, :3] = Rotation.from_quat(quaternions).as_matrix()
    initial[:, :3, 3] = positions
    corrected = corrections @ initial
    return corrected[:, :3, 3], Rotation.from_matrix(corrected[:, :3, :3]).as_quat()
```

`matrices_at` builds homogeneous matrices from existing interpolated correction
translations and quaternions.

- [ ] **Step 4: Run focused and existing optimized-mapping tests**

Run: `python3 -m pytest -q tests/test_optimized_pose_correction.py tests/test_optimized_mapping.py`

Expected: all tests pass.

- [ ] **Step 5: Record checkpoint**

Record changed files and passing test counts; no Git commit is available.

### Task 3: Bounded Multi-View Color Projection

**Files:**
- Create: `survey_mapping/color_fusion.py`
- Create: `tests/test_color_fusion.py`

**Interfaces:**
- Produces dataclass: `CameraView(lidar_time, image_bgr, position_world_body, quaternion_world_body)`.
- Produces: `TimeOrderedCameraWindow(samples, window_s, maximum_candidates)`.
- Produces: `candidates(target_time) -> list[tuple[float, ndarray]]` ordered by absolute time difference.
- Produces: `colorize_optimized_points(points_world, reference_time, camera_views, r_bl, t_bl, r_cl, t_cl, intrinsics, distortion, depth_tolerance) -> tuple[rgb, color_valid, statistics]`.

- [ ] **Step 1: Write failing camera-window and projection tests**

```python
def test_camera_window_returns_three_nearest_samples():
    samples = iter([(0.0, "a"), (0.1, "b"), (0.2, "c"), (0.3, "d")])
    window = TimeOrderedCameraWindow(samples, window_s=0.12, maximum_candidates=3)
    assert [item[1] for item in window.candidates(0.15)] == ["b", "c", "a"]


def test_multiview_prefers_nearest_visible_camera():
    # Identity calibration, one point at camera depth 5 m, red nearest image,
    # green farther image. The nearest valid observation must win.
    rgb, valid, stats = colorize_optimized_points(
        np.array([[0.0, 0.0, 5.0]]),
        reference_time=1.0,
        camera_views=[far_green_view, near_red_view],
        rotation_body_lidar=np.eye(3),
        translation_body_lidar=np.zeros(3),
        rotation_camera_lidar=np.eye(3),
        translation_camera_lidar=np.zeros(3),
        intrinsics=np.array([[100.0, 0.0, 1.0], [0.0, 100.0, 1.0], [0.0, 0.0, 1.0]]),
        distortion=np.zeros(4),
        depth_tolerance=0.3,
    )
    assert valid.tolist() == [True]
    assert rgb.tolist() == [[255, 0, 0]]
```

- [ ] **Step 2: Run tests and verify RED**

Run: `python3 -m pytest -q tests/test_color_fusion.py`

Expected: collection fails because `survey_mapping.color_fusion` does not exist.

- [ ] **Step 3: Implement the bounded camera window**

Keep one pending future sample and a deque of decoded samples. For target `t`,
read until the pending sample exceeds `t + window_s`, discard samples before
`t - window_s`, sort by `abs(sample_time - t)`, and return at most three.

- [ ] **Step 4: Implement conservative multi-view projection**

For each `CameraView` in increasing absolute time difference:

```python
rotation_world_body = Rotation.from_quat(view.quaternion_world_body)
points_body = rotation_world_body.inv().apply(points_world - view.position_world_body)
points_lidar = (rotation_body_lidar.T @ (points_body - translation_body_lidar).T).T
uv, depth, valid = project_lidar_points(
    points_lidar,
    rotation_camera_lidar,
    translation_camera_lidar,
    intrinsics,
    distortion,
    view.image_bgr.shape,
)
visible = visibility_mask(uv, depth, valid, view.image_bgr.shape, depth_tolerance)
unassigned = visible & ~color_valid
rgb[unassigned] = sample_bgr_bilinear(view.image_bgr, uv[unassigned])[:, ::-1]
color_valid[unassigned] = True
```

Do not replace an already assigned color with a farther camera observation.
Return projected and visible counts per view in `statistics`.

- [ ] **Step 5: Run focused projection tests**

Run: `python3 -m pytest -q tests/test_color_fusion.py tests/test_camera_projection.py`

Expected: all tests pass.

- [ ] **Step 6: Record checkpoint**

Record changed files and test output; no Git commit is available.

### Task 4: Integrate Livox Deskew and Dual Color Outputs

**Files:**
- Modify: `scripts/build_optimized_color_cloud.py`
- Create: `tests/test_optimized_color_pipeline.py`

**Interfaces:**
- Consumes: `load_optimized_model(output_root)` returning navigation,
  navigation pose series, correction interpolator, and correction times.
- Consumes: `prepare_deskew(config, extracted_root)` returning measured time
  alignment and Livox IMU rotations.
- Produces: `build_color_cloud(config) -> dict` and `full_color_report.json`.

- [ ] **Step 1: Write failing configuration and report-contract tests**

```python
def test_color_output_paths_are_distinct(tmp_path):
    config = make_color_config(tmp_path)
    complete, strict = color_output_paths(config)
    assert complete.name == "cloud_optimized_color_complete.ply"
    assert strict.name == "cloud_optimized_color_strict.ply"
    assert complete != strict


def test_full_mode_rejects_active_frame_limits():
    config = make_color_config(Path("out"))
    config["camera"]["full_bag_required"] = True
    config["mapping"]["max_lidar_frames"] = 120
    with pytest.raises(ValueError, match="full-bag colorization forbids"):
        validate_color_config(config)
```

- [ ] **Step 2: Run contract tests and verify RED**

Run: `python3 -m pytest -q tests/test_optimized_color_pipeline.py`

Expected: import failure for the new validation/path helpers.

- [ ] **Step 3: Implement exact validation and output helpers**

`validate_color_config` must reject duplicate output paths, unsupported deskew
mode, correlation fallback in production mode, and active frame limits when
`camera.full_bag_required: true`.

- [ ] **Step 4: Replace navigation-only point projection with Livox deskew**

For each LiDAR frame, use the same sequence as `build_optimized_cloud.py`:

```python
reference_time = frame_time + float(np.max(points[:, 0])) * 1e-9
deskewed = deskew_points_with_imu(
    points, frame_time, reference_time, imu_rotations, pose_series,
    alignment.offset_s, r_bl, t_bl, r_il, t_il,
)
points_body_reference = (r_bl @ deskewed[:, :3].T).T + t_bl
reference_times = np.full(len(points), reference_time)
points_world_initial = navigation.transform_body_points(
    points_body_reference, reference_times + alignment.offset_s
)
points_world_optimized = correction.apply(points_world_initial, reference_times)
```

- [ ] **Step 5: Build corrected camera views and write both clouds**

For each selected camera sample at adjusted Livox time `t_camera`:

```python
initial_position, initial_quaternion = navigation.sample_many(
    np.array([t_camera + alignment.offset_s])
)
position, quaternion = correction.correct_poses(
    initial_position, initial_quaternion, np.array([t_camera])
)
view = CameraView(t_camera, image, position[0], quaternion[0])
```

Call `colorize_optimized_points`, derive intensity bytes from Livox
reflectivity, and pass all arrays to `DualColorPlyWriter.write`.

- [ ] **Step 6: Write the complete report contract**

Report exact keys:

```python
report = {
    "uses_mapping_fusion": False,
    "uses_fast_lio": False,
    "deskew_orientation_source": "livox_imu",
    "navigation_time_offset_s": float(alignment.offset_s),
    "time_alignment_correlation": float(alignment.correlation),
    "lidar_frame_count": frame_count,
    "paired_camera_frame_count": paired_frames,
    "input_point_count": input_points,
    "complete_point_count": writer.complete_count,
    "strict_point_count": writer.strict_count,
    "true_color_point_count": writer.true_color_count,
    "true_color_fraction": writer.true_color_count / writer.complete_count,
    "paired_frame_fraction": paired_frames / frame_count,
    "mean_nearest_camera_time_difference_s": float(np.mean(nearest_time_differences)),
    "max_nearest_camera_time_difference_s": float(np.max(nearest_time_differences)),
    "complete_cloud_path": str(complete_path),
    "strict_cloud_path": str(strict_path),
}
```

- [ ] **Step 7: Run focused and full unit tests**

Run: `python3 -m pytest -q tests/test_optimized_color_pipeline.py tests/test_color_output.py tests/test_color_fusion.py`

Run: `python3 -m pytest -q`

Expected: all tests pass.

- [ ] **Step 8: Record checkpoint**

Record changed files and test counts; no Git commit is available.

### Task 5: Independent Color-Cloud Verifier

**Files:**
- Create: `scripts/verify_color_cloud.py`
- Create: `tests/test_verify_color_cloud.py`

**Interfaces:**
- Produces: `verify_complete_ply(path) -> dict`.
- Produces: `verify_strict_ply(path) -> dict`.
- Complete result includes declared/actual count, finite count, true-color
  count, bounds, and file size.

- [ ] **Step 1: Write failing verification tests**

Create files with the Task 1 writer, verify a valid pair, then truncate one
binary record and require `ValueError("PLY declared vertex count does not match")`.

- [ ] **Step 2: Run verifier tests and verify RED**

Run: `python3 -m pytest -q tests/test_verify_color_cloud.py`

Expected: import failure for `scripts.verify_color_cloud`.

- [ ] **Step 3: Implement exact binary verification**

Parse through `end_header`, require the expected properties in order, calculate
record count from payload bytes and dtype item size, load records with
`np.fromfile`, reject non-finite XYZ, and return min/max bounds. For the complete
cloud, require every `color_valid` value to be `0` or `1`.

- [ ] **Step 4: Run verifier and writer tests**

Run: `python3 -m pytest -q tests/test_verify_color_cloud.py tests/test_color_output.py`

Expected: all tests pass.

- [ ] **Step 5: Record checkpoint**

Record changed files and results; no Git commit is available.

### Task 6: Sample Configuration and Acceptance Run

**Files:**
- Create: `config/HKisland_GNSS02_joint_sample_livox_deskew_color.yaml`
- Create: `docs/2026-07-21-sample-color-map-results.md`

**Interfaces:**
- Output root: `outputs_joint_kfgins_sample_livox_deskew_color`.
- Uses frames 500 through 619, 72,000-point input cap, keyframe stride 5.
- Produces 120-frame geometry, complete color, strict color, and reports.

- [ ] **Step 1: Create the sample configuration**

Copy the validated sample Livox-deskew configuration and set:

```yaml
outputs:
  root: outputs_joint_kfgins_sample_livox_deskew_color
camera:
  complete_output: outputs_joint_kfgins_sample_livox_deskew_color/cloud_optimized_color_complete.ply
  strict_output: outputs_joint_kfgins_sample_livox_deskew_color/cloud_optimized_color_strict.ply
  multi_view_window_s: 0.12
  maximum_camera_candidates: 3
  minimum_true_color_fraction: 0.10
  minimum_paired_frame_fraction: 0.95
  full_bag_required: false
```

- [ ] **Step 2: Reuse only same-bag extracted observations**

Copy `extracted/*.csv` from the validated output cache into the new sample
output root. Do not copy keyframes, optimized poses, geometry, or colors.

- [ ] **Step 3: Rebuild sample geometry and optimization**

Run:

`python3 scripts/build_lidar_keyframes.py --config config/HKisland_GNSS02_joint_sample_livox_deskew_color.yaml`

`python3 scripts/build_lidar_constraints.py --config config/HKisland_GNSS02_joint_sample_livox_deskew_color.yaml`

`python3 scripts/optimize_lidar_pose_graph.py --config config/HKisland_GNSS02_joint_sample_livox_deskew_color.yaml`

`python3 scripts/build_optimized_cloud.py --config config/HKisland_GNSS02_joint_sample_livox_deskew_color.yaml`

Expected: 120 selected scans, 24 keyframes, finite geometry, and 23 accepted
adjacent LiDAR edges unless a measured rejection is explained in the report.

- [ ] **Step 4: Build and verify sample color clouds**

Run:

`python3 scripts/build_optimized_color_cloud.py --config config/HKisland_GNSS02_joint_sample_livox_deskew_color.yaml`

`python3 scripts/verify_color_cloud.py --config config/HKisland_GNSS02_joint_sample_livox_deskew_color.yaml`

Require: complete count equals geometry count, strict count equals complete
`color_valid == 1`, paired-frame fraction at least `0.95`, mean nearest-camera
time difference at most `0.03 s`, and true-color fraction at least `0.10`.

- [ ] **Step 5: Record measured sample results**

Write exact counts, coverage, timing, bounds, file sizes, and any gate failure to
`docs/2026-07-21-sample-color-map-results.md`.

### Task 7: Full Configuration and Complete-Bag Run

**Files:**
- Create: `config/HKisland_GNSS02_joint_all_livox_deskew_color.yaml`
- Create: `docs/2026-07-21-full-color-map-results.md`

**Interfaces:**
- Output root: `outputs_joint_kfgins_all_livox_deskew_color`.
- Produces all deliverables named in the design specification.

- [ ] **Step 1: Check storage before the full run**

Run: `df -h /home/shuai/cehui`

Require at least `8 GiB` free before starting. If less is available, stop and
report the measured free space without deleting existing outputs.

- [ ] **Step 2: Create the full configuration**

Start from the sample configuration, change the output root and output paths,
set `keyframe_stride: 10`, set `camera.full_bag_required: true`, and omit
`start_lidar_frame`, `max_lidar_frames`, and `max_points_per_frame` entirely.

- [ ] **Step 3: Reuse only same-bag extracted observations**

Copy the validated extracted CSV cache into the new full output root. Keep every
mapping, optimization, geometry, and color output absent before the run.

- [ ] **Step 4: Rebuild the complete optimized geometry**

Run the keyframe, constraint, optimization, hardware time-sync, and optimized
geometry commands from Task 6 with the full configuration. Wait for each
process to finish and verify its report before starting the next process.

- [ ] **Step 5: Build both complete-bag color clouds**

Run:

`python3 scripts/build_optimized_color_cloud.py --config config/HKisland_GNSS02_joint_all_livox_deskew_color.yaml`

Do not stop the process while a writer is open. Preserve progress logs every
100 LiDAR frames.

- [ ] **Step 6: Verify all full outputs independently**

Run:

`python3 scripts/verify_color_cloud.py --config config/HKisland_GNSS02_joint_all_livox_deskew_color.yaml`

Run: `python3 -m pytest -q`

Require all full-run acceptance gates from the design. Report any unmet gate as
an actual limitation; do not rename a partial file as a complete result.

- [ ] **Step 7: Write the full results document**

Record processed LiDAR frames, raw/accepted points, complete/strict counts,
true-color fraction, paired-frame fraction, time differences, coordinate
bounds, PLY file sizes, alignment evidence, LiDAR edge statistics, and exact
absolute output paths in `docs/2026-07-21-full-color-map-results.md`.

## Plan Self-Review

- Spec coverage: geometry, time alignment, optimized camera poses, multi-view
  projection, z-buffer visibility, bilinear RGB, complete/strict outputs,
  streaming, sample gates, full run, and independent verification are assigned
  to Tasks 1 through 7.
- Placeholder scan: no deferred placeholder text or unspecified implementation
  steps remain.
- Type consistency: `CameraView`, `DualColorPlyWriter`, correction methods,
  output helper names, report keys, and PLY layouts are consistent across tasks.
- Scope: color-map completion is one dependent pipeline; it should remain one
  plan because the full run cannot be accepted before all earlier tasks pass.
