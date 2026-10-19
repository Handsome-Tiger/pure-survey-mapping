#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from survey_mapping.ekf import GnssInsFilter
from survey_mapping.geo import geodetic_to_enu, quat_to_matrix
from survey_mapping.io import read_numeric_csv, write_csv, write_json
from survey_mapping.livox import filter_valid_points, georeference_points, write_ply
from survey_mapping.trajectory import build_pose_series

from extract_bag import extract


def load_extracted(root):
    root = Path(root)
    _, gnss = read_numeric_csv(root / "gnss.csv")
    _, imu = read_numeric_csv(root / "imu.csv")
    _, attitude = read_numeric_csv(root / "attitude.csv")
    lidar = np.load(root / "livox_frames.npz")
    return gnss, imu, attitude, lidar


def run_filter(config, gnss, imu, attitude):
    origin = tuple(gnss[0, 1:4])
    gnss_times = gnss[:, 0]
    gnss_enu = np.asarray(
        [geodetic_to_enu(lat, lon, alt, origin) for lat, lon, alt in gnss[:, 1:4]],
        dtype=float,
    )
    imu_times = imu[:, 0]
    imu_accels_body = imu[:, 4:7]
    attitude_times = attitude[:, 0] if len(attitude) else None
    quats = attitude[:, 1:5] if len(attitude) else None

    filt = GnssInsFilter(
        accel_noise=float(config["filter"].get("accel_noise", 0.8)),
        gnss_sigma=float(config["filter"].get("gnss_sigma_m", 1.5)),
    )
    filt.reset(gnss_enu[0])
    trajectory_rows = []
    gnss_idx = 0
    last_t = float(gnss_times[0])
    for idx, (t, accel_body) in enumerate(zip(imu_times, imu_accels_body)):
        if t < gnss_times[0]:
            continue
        if quats is not None:
            q = quats[min(np.searchsorted(attitude_times, t), len(quats) - 1)]
            accel_map = quat_to_matrix(q) @ accel_body
        else:
            accel_map = accel_body
        if config["filter"].get("subtract_gravity", True):
            accel_map = accel_map - np.array([0.0, 0.0, 9.80665])
        filt.predict(float(t) - last_t, accel_map)
        last_t = float(t)
        residual = np.array([np.nan, np.nan, np.nan])
        while gnss_idx < len(gnss_times) and gnss_times[gnss_idx] <= t:
            residual = filt.update_gnss(gnss_enu[gnss_idx])
            gnss_idx += 1
        state = filt.state()
        trajectory_rows.append([t, *state.position.tolist(), *state.velocity.tolist(), *residual.tolist()])
    trajectory = np.asarray(trajectory_rows, dtype=float)
    return origin, gnss_enu, trajectory


def build_cloud(config, trajectory, attitude, lidar):
    out_root = Path(config["outputs"]["root"])
    times = trajectory[:, 0]
    positions = trajectory[:, 1:4]
    attitude_times = attitude[:, 0] if len(attitude) else None
    quats = attitude[:, 1:5] if len(attitude) else None
    pose_series = build_pose_series(times, positions, attitude_times, quats)
    rotation = np.asarray(config["mapping"]["extrinsic"]["rotation_body_lidar"], dtype=float)
    translation = np.asarray(config["mapping"]["extrinsic"]["translation_body_lidar_m"], dtype=float)
    cloud_parts = []
    stamps = lidar["stamps"]
    for idx, stamp in enumerate(stamps):
        points = lidar[f"points_{idx}"]
        valid = filter_valid_points(points)
        cloud_parts.append(
            georeference_points(valid, float(stamp), pose_series, rotation, translation)
        )
    cloud = np.vstack(cloud_parts) if cloud_parts else np.zeros((0, 4))
    cloud_path = out_root / "cloud_sample.ply"
    write_ply(cloud_path, cloud)
    return cloud_path, len(cloud)


def run(config):
    extracted_root = Path(config["outputs"]["root"]) / "extracted"
    extract(config)
    gnss, imu, attitude, lidar = load_extracted(extracted_root)
    origin, gnss_enu, trajectory = run_filter(config, gnss, imu, attitude)
    out_root = Path(config["outputs"]["root"])
    write_csv(
        out_root / "trajectory_fused.csv",
        ["time", "x", "y", "z", "vx", "vy", "vz", "res_x", "res_y", "res_z"],
        trajectory,
    )
    write_csv(
        out_root / "trajectory_gnss.csv",
        ["time", "x", "y", "z"],
        np.column_stack([gnss[:, 0], gnss_enu]),
    )
    cloud_path, point_count = build_cloud(config, trajectory, attitude, lidar)
    report = {
        "origin_lat_lon_h": [float(v) for v in origin],
        "gnss_count": int(len(gnss)),
        "imu_count": int(len(imu)),
        "attitude_count": int(len(attitude)),
        "trajectory_count": int(len(trajectory)),
        "cloud_point_count": int(point_count),
        "cloud_path": str(cloud_path),
        "bag_path": str(config["bag"]["path"]),
        "start_lidar_frame": int(config["mapping"].get("start_lidar_frame", 0)),
        "max_lidar_frames": int(config["mapping"].get("max_lidar_frames", 1)),
    }
    write_json(out_root / "report.json", report)
    return report


def main():
    parser = argparse.ArgumentParser(description="Run GNSS/INS direct LiDAR georeferencing.")
    parser.add_argument("--config", default="config/default.yaml")
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    report = run(config)
    for key, value in report.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
