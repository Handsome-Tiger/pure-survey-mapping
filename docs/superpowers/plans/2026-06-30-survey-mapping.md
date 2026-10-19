# Survey Mapping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a runnable pure survey mapping MVP for `HKisland_GNSS02_60s.bag`.

**Architecture:** Implement pure math and filtering modules first, then connect them to ROS1 bag raw-message extraction. Keep all core behavior testable without ROS message packages.

**Tech Stack:** Python 3.10, NumPy, pytest, rosbag raw reads, YAML, CSV, PLY output.

---

### Task 1: Core Math Tests

**Files:**
- Create: `tests/test_geo.py`
- Create: `tests/test_rosraw.py`
- Create: `tests/test_ekf.py`
- Create: `tests/test_trajectory_livox.py`

- [ ] Write tests for geodetic ENU conversion, quaternion rotation, ROS raw message parsing, EKF GNSS correction, trajectory interpolation, and PLY export.
- [ ] Run `python3 -m pytest tests -q` and verify tests fail because modules do not exist.

### Task 2: Core Package

**Files:**
- Create: `survey_mapping/__init__.py`
- Create: `survey_mapping/geo.py`
- Create: `survey_mapping/rosraw.py`
- Create: `survey_mapping/ekf.py`
- Create: `survey_mapping/trajectory.py`
- Create: `survey_mapping/livox.py`
- Create: `survey_mapping/io.py`

- [ ] Implement only the behavior needed by the tests.
- [ ] Run `python3 -m pytest tests -q` and verify all tests pass.

### Task 3: Bag Extraction

**Files:**
- Create: `scripts/extract_bag.py`
- Create: `config/default.yaml`

- [ ] Use `rosbag.Bag(...).read_messages(raw=True)` so the extractor does not depend on installed ROS message Python packages.
- [ ] Parse `/dji_osdk_ros/rtk_position`, `/dji_osdk_ros/imu`, `/dji_osdk_ros/attitude`, and `/livox/lidar`.
- [ ] Write compact CSV/NPZ outputs for downstream processing.

### Task 4: Mapping Pipeline

**Files:**
- Create: `scripts/run_mapping.py`
- Create: `README.md`

- [ ] Load extracted observations.
- [ ] Run GNSS/INS fusion.
- [ ] Interpolate fused trajectory and attitude.
- [ ] Georeference a configurable number of Livox points to `outputs/cloud_sample.ply`.
- [ ] Write a JSON report with counts, time span, origin, and output paths.

### Task 5: Verification

**Files:**
- Modify: `README.md`

- [ ] Run unit tests.
- [ ] Run the pipeline on a small point limit from `HKisland_GNSS02_60s.bag`.
- [ ] Confirm CSV, PLY, and report outputs exist and contain plausible data.
