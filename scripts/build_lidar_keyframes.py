#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from scripts.run_mapping_full_stream import (
    build_mapping_pose_series,
    extract_navigation,
    lidar_limits,
    raw_bytes,
    run_filter_error_state,
)
from survey_mapping.geo import quat_to_matrix
from survey_mapping.imu_deskew import ImuRotationSeries, deskew_points_with_imu
from survey_mapping.io import read_numeric_csv, write_csv, write_json
from survey_mapping.lidar_odometry import deskew_points, voxel_centroids
from survey_mapping.livox import filter_valid_points
from survey_mapping.pipeline_alignment import resolve_time_alignment
from survey_mapping.rosraw import parse_livox_custom


def load_filter_inputs(extracted_root):
    names = ("gnss", "imu", "attitude", "rtk_velocity", "rtk_yaw", "rtk_quality")
    return [read_numeric_csv(Path(extracted_root) / f"{name}.csv")[1] for name in names]


def build_navigation(config):
    output_root = Path(config["outputs"]["root"])
    extracted_root = output_root / "extracted"
    required = [
        extracted_root / f"{name}.csv"
        for name in (
            "gnss",
            "imu",
            "attitude",
            "rtk_velocity",
            "rtk_yaw",
            "rtk_quality",
            "livox_imu",
        )
    ]
    if not all(path.exists() for path in required):
        extracted_root = extract_navigation(config)
    gnss, imu, attitude, rtk_velocity, rtk_yaw, rtk_quality = load_filter_inputs(
        extracted_root
    )
    origin, gnss_enu, trajectory, residuals, biases = run_filter_error_state(
        config, gnss, imu, attitude, rtk_velocity, rtk_yaw, rtk_quality
    )
    output_root.mkdir(parents=True, exist_ok=True)
    write_csv(
        output_root / "trajectory_nav_initial.csv",
        [
            "time",
            "x",
            "y",
            "z",
            "vx",
            "vy",
            "vz",
            "qx",
            "qy",
            "qz",
            "qw",
            "res_x",
            "res_y",
            "res_z",
        ],
        trajectory,
    )
    write_csv(
        output_root / "trajectory_gnss.csv",
        ["time", "x", "y", "z"],
        np.column_stack([gnss[:, 0], gnss_enu]),
    )
    write_csv(
        output_root / "gnss_update_residual.csv",
        ["time", "res_x", "res_y", "res_z", "res_3d", "accepted"],
        residuals,
    )
    write_csv(
        output_root / "imu_bias.csv",
        ["time", "bg_x", "bg_y", "bg_z", "ba_x", "ba_y", "ba_z"],
        biases,
    )
    return origin, trajectory, attitude, extracted_root


def prepare_deskew(config, extracted_root):
    output_root = Path(config["outputs"]["root"])
    _header, dji_imu = read_numeric_csv(Path(extracted_root) / "imu.csv")
    _header, livox_imu = read_numeric_csv(Path(extracted_root) / "livox_imu.csv")
    alignment = resolve_time_alignment(config, dji_imu, livox_imu)
    report = {
        "offset_convention": "t_navigation = t_livox + offset_s",
        "offset_s": float(alignment.offset_s),
        "correlation": (
            float(alignment.correlation) if np.isfinite(alignment.correlation) else None
        ),
        "margin": float(alignment.margin) if np.isfinite(alignment.margin) else None,
        "sample_count": int(alignment.sample_count),
        "method": alignment.method,
        "used_fallback": bool(alignment.used_fallback),
        "dji_imu_count": int(len(dji_imu)),
        "livox_imu_count": int(len(livox_imu)),
    }
    write_json(output_root / "time_alignment_report.json", report)
    bias = np.asarray(
        config.get("mapping", {}).get("livox_imu_gyro_bias_rad_s", [0.0, 0.0, 0.0]),
        dtype=float,
    )
    rotations = ImuRotationSeries.from_gyro(livox_imu[:, 0], livox_imu[:, 1:4], bias)
    return alignment, rotations


def build_keyframes(config):
    import rosbag

    output_root = Path(config["outputs"]["root"])
    keyframe_root = output_root / "keyframes"
    keyframe_root.mkdir(parents=True, exist_ok=True)
    origin, trajectory, attitude, extracted_root = build_navigation(config)
    pose_series = build_mapping_pose_series(config, trajectory, attitude)
    alignment, imu_rotations = prepare_deskew(config, extracted_root)
    mapping = config.get("mapping", {})
    lidar_topic = config["bag"]["lidar_topic"]
    r_bl = np.asarray(mapping["extrinsic"]["rotation_body_lidar"], dtype=float)
    t_bl = np.asarray(mapping["extrinsic"]["translation_body_lidar_m"], dtype=float)
    imu_extrinsic = mapping.get("livox_imu_extrinsic", {})
    r_il = np.asarray(imu_extrinsic.get("rotation_imu_lidar", np.eye(3)), dtype=float)
    t_il = np.asarray(
        imu_extrinsic.get("translation_imu_lidar_m", [0.0, 0.0, 0.0]), dtype=float
    )
    deskew_source = str(mapping.get("deskew_orientation_source", "navigation")).lower()
    if deskew_source not in ("navigation", "livox_imu"):
        raise ValueError(
            "mapping.deskew_orientation_source must be 'navigation' or 'livox_imu'"
        )
    stride = int(mapping.get("keyframe_stride", 5))
    voxel_size = float(mapping.get("registration_voxel_m", 0.5))
    blind = float(mapping.get("blind_m", 0.8))
    navigation_time_offset = float(alignment.offset_s)
    start_frame, max_frames, max_points = lidar_limits(config)
    valid_seen = 0
    selected_seen = 0
    rows = []

    with rosbag.Bag(str(config["bag"]["path"])) as bag:
        for _topic, raw_msg, _stamp in bag.read_messages(topics=[lidar_topic], raw=True):
            msg = parse_livox_custom(raw_bytes(raw_msg))
            points = filter_valid_points(msg.points, min_range=blind)
            if not len(points):
                continue
            if valid_seen < start_frame:
                valid_seen += 1
                continue
            if max_frames is not None and selected_seen >= max_frames:
                break
            frame_index = selected_seen
            selected_seen += 1
            valid_seen += 1
            if frame_index % stride != 0:
                continue
            if max_points is not None:
                points = points[:max_points]
            frame_time = msg.timebase_ns * 1e-9
            reference_time = frame_time + float(np.max(points[:, 0])) * 1e-9
            if deskew_source == "livox_imu":
                deskewed = deskew_points_with_imu(
                    points,
                    frame_time,
                    reference_time,
                    imu_rotations,
                    pose_series,
                    navigation_time_offset,
                    r_bl,
                    t_bl,
                    r_il,
                    t_il,
                )
            else:
                deskewed = deskew_points(
                    points,
                    frame_time,
                    pose_series,
                    reference_time,
                    r_bl,
                    t_bl,
                    navigation_time_offset,
                )
            points_local = voxel_centroids(deskewed[:, :3], voxel_size)
            pose = pose_series.sample(reference_time + navigation_time_offset)
            r_wb = quat_to_matrix(pose.quaternion)
            transform = np.eye(4, dtype=float)
            transform[:3, :3] = r_wb @ r_bl
            transform[:3, 3] = pose.position + r_wb @ t_bl
            keyframe_index = len(rows)
            path = keyframe_root / f"kf_{keyframe_index:05d}.npz"
            np.savez_compressed(
                path,
                time=np.array(reference_time),
                frame_index=np.array(frame_index),
                transform_initial=transform,
                points_local=points_local.astype(np.float32),
            )
            rows.append(
                [
                    keyframe_index,
                    frame_index,
                    reference_time,
                    len(points),
                    len(points_local),
                    str(path),
                ]
            )

    write_csv(
        output_root / "keyframes.csv",
        [
            "keyframe_index",
            "lidar_frame_index",
            "time",
            "raw_points",
            "registration_points",
            "path",
        ],
        rows,
    )
    report = {
        "origin_lat_lon_h": [float(value) for value in origin],
        "selected_lidar_frames": int(selected_seen),
        "keyframe_count": int(len(rows)),
        "keyframe_stride": stride,
        "registration_voxel_m": voxel_size,
        "blind_m": blind,
        "deskew_orientation_source": deskew_source,
        "navigation_time_offset_s": navigation_time_offset,
        "time_alignment_correlation": (
            float(alignment.correlation) if np.isfinite(alignment.correlation) else None
        ),
        "rotation_imu_lidar": r_il.tolist(),
        "translation_imu_lidar_m": t_il.tolist(),
    }
    write_json(output_root / "keyframe_report.json", report)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    report = build_keyframes(config)
    for key, value in report.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
