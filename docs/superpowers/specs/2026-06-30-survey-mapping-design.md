# Pure Survey Mapping Project Design

## Goal

Build a runnable offline mapping project for `HKisland_GNSS02_60s.bag` that follows a survey-production workflow: GNSS/RTK position constraints plus high-rate inertial propagation, then direct LiDAR georeferencing.

## Data Context

The test bag is 60 seconds long and contains Livox LiDAR, Livox IMU, DJI IMU, DJI RTK/GNSS, DJI attitude, and u-blox GNSS topics. No base-station RINEX file is present in the workspace, so the first runnable version treats `/dji_osdk_ros/rtk_position` or `/ublox_driver/receiver_lla` as the GNSS position observation stream. RTKLIB/PPK is documented as an extension point.

## Architecture

The project separates survey math from ROS bag I/O.

- `survey_mapping.geo`: WGS84 geodetic to local ENU conversions.
- `survey_mapping.rosraw`: small ROS1 raw-message parsers for only the message types needed by this bag.
- `survey_mapping.ekf`: a lightweight loosely coupled GNSS/INS filter for position and velocity.
- `survey_mapping.trajectory`: timestamped pose interpolation.
- `survey_mapping.livox`: Livox point parsing and direct georeferencing.
- `scripts/extract_bag.py`: extract GNSS, IMU, attitude, and Livox samples from the bag.
- `scripts/run_mapping.py`: run extraction, fusion, and point cloud generation.

## Fusion Design

The MVP uses a loosely coupled filter. IMU acceleration propagates position and velocity at high rate. GNSS/RTK positions update the filter at lower rate. GNSS observation covariance is configurable because the bag covariance fields are zero. The design directly addresses the frequency and accuracy mismatch: GNSS constrains long-term drift, while IMU fills high-frequency motion between GNSS epochs.

Orientation for point cloud georeferencing comes from DJI attitude if available. This keeps the project useful with the current bag because the Livox IMU orientation field is zero. The filter still uses IMU accelerations for propagation; attitude is sampled separately to rotate LiDAR points into the map frame.

## Mapping Design

Each Livox point timestamp is computed from `timebase + offset_time`. The mapper interpolates the fused pose at each point time and applies:

`P_map = R_map_body(t) * (R_body_lidar * P_lidar + t_body_lidar) + p_map_body(t)`

The default extrinsic is identity and must be calibrated for survey-grade output. The project exposes the extrinsic in YAML so later boresight and lever-arm calibration can be added without changing code.

## Outputs

- `outputs/extracted/gnss.csv`
- `outputs/extracted/imu.csv`
- `outputs/extracted/attitude.csv`
- `outputs/trajectory_fused.csv`
- `outputs/cloud_sample.ply`
- `outputs/report.json`

## Limits

This MVP is a test-data pipeline and algorithm scaffold, not a certified survey adjustment package. It does not solve raw GNSS carrier phase, run RTKLIB without base data, estimate boresight, classify ground, or generate DEM/DSM. Those are preserved as documented extension steps matching the reference document.
