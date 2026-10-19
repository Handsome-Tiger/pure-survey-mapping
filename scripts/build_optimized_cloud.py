#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from scripts.build_lidar_constraints import load_keyframes
from scripts.build_lidar_keyframes import prepare_deskew
from scripts.run_mapping_full_stream import lidar_limits, raw_bytes
from survey_mapping.imu_deskew import deskew_points_with_imu
from survey_mapping.io import read_numeric_csv, write_json
from survey_mapping.livox import filter_valid_points
from survey_mapping.optimized_mapping import (
    BinaryPlyWriter,
    PoseInterpolator,
    TransformCorrectionInterpolator,
)
from survey_mapping.rosraw import parse_livox_custom
from survey_mapping.trajectory import build_pose_series


def load_optimized_model(output_root):
    output_root = Path(output_root)
    _header, trajectory = read_numeric_csv(output_root / "trajectory_nav_initial.csv")
    if trajectory.shape[1] < 11:
        raise ValueError("trajectory_nav_initial.csv must contain quaternion columns")
    navigation = PoseInterpolator(
        trajectory[:, 0], trajectory[:, 1:4], trajectory[:, 7:11]
    )
    pose_series = build_pose_series(
        trajectory[:, 0], trajectory[:, 1:4], trajectory[:, 0], trajectory[:, 7:11]
    )

    keyframes = load_keyframes(output_root)
    optimized = np.load(output_root / "keyframe_poses_optimized.npz")["transforms"]
    if len(keyframes) != len(optimized):
        raise ValueError("Initial and optimized keyframe counts differ")
    times = np.asarray([keyframe["time"] for keyframe in keyframes], dtype=float)
    corrections = np.stack(
        [after @ np.linalg.inv(before["transform"]) for before, after in zip(keyframes, optimized)]
    )
    return (
        navigation,
        pose_series,
        TransformCorrectionInterpolator(times, corrections),
        times,
    )


def build_cloud(config):
    import rosbag

    output_root = Path(config["outputs"]["root"])
    navigation, pose_series, correction, correction_times = load_optimized_model(output_root)
    alignment, imu_rotations = prepare_deskew(config, output_root / "extracted")
    mapping = config.get("mapping", {})
    rotation_body_lidar = np.asarray(
        mapping["extrinsic"]["rotation_body_lidar"], dtype=float
    )
    translation_body_lidar = np.asarray(
        mapping["extrinsic"]["translation_body_lidar_m"], dtype=float
    )
    imu_extrinsic = mapping.get("livox_imu_extrinsic", {})
    rotation_imu_lidar = np.asarray(
        imu_extrinsic.get("rotation_imu_lidar", np.eye(3)), dtype=float
    )
    translation_imu_lidar = np.asarray(
        imu_extrinsic.get("translation_imu_lidar_m", [0.0, 0.0, 0.0]), dtype=float
    )
    deskew_source = str(mapping.get("deskew_orientation_source", "navigation")).lower()
    if deskew_source not in ("navigation", "livox_imu"):
        raise ValueError(
            "mapping.deskew_orientation_source must be 'navigation' or 'livox_imu'"
        )
    blind = float(mapping.get("blind_m", 0.8))
    navigation_time_offset = float(alignment.offset_s)
    start_frame, maximum_frames, maximum_points = lidar_limits(config)
    lidar_topic = config["bag"]["lidar_topic"]
    output_path = Path(
        mapping.get("optimized_cloud_output", output_root / "cloud_optimized_geometry.ply")
    )

    valid_seen = 0
    frame_count = 0
    raw_point_count = 0
    written = 0
    first_time = None
    last_time = None
    with BinaryPlyWriter(output_path, with_rgb=False) as writer:
        with rosbag.Bag(str(config["bag"]["path"])) as bag:
            for _topic, raw_message, _stamp in bag.read_messages(topics=[lidar_topic], raw=True):
                message = parse_livox_custom(raw_bytes(raw_message))
                points = filter_valid_points(message.points, min_range=blind)
                if not len(points):
                    continue
                if valid_seen < start_frame:
                    valid_seen += 1
                    continue
                if maximum_frames is not None and frame_count >= maximum_frames:
                    break
                valid_seen += 1
                if maximum_points is not None:
                    points = points[:maximum_points]

                frame_time = message.timebase_ns * 1e-9
                point_times = frame_time + points[:, 0] * 1e-9
                if deskew_source == "livox_imu":
                    reference_time = float(point_times[-1])
                    deskewed = deskew_points_with_imu(
                        points,
                        frame_time,
                        reference_time,
                        imu_rotations,
                        pose_series,
                        navigation_time_offset,
                        rotation_body_lidar,
                        translation_body_lidar,
                        rotation_imu_lidar,
                        translation_imu_lidar,
                    )
                    points_body = (
                        rotation_body_lidar @ deskewed[:, :3].T
                    ).T + translation_body_lidar
                    reference_times = np.full(len(points), reference_time, dtype=float)
                    points_world = navigation.transform_body_points(
                        points_body, reference_times + navigation_time_offset
                    )
                    points_optimized = correction.apply(points_world, reference_times)
                else:
                    points_body = (
                        rotation_body_lidar @ points[:, 1:4].T
                    ).T + translation_body_lidar
                    points_world = navigation.transform_body_points(
                        points_body, point_times + navigation_time_offset
                    )
                    points_optimized = correction.apply(points_world, point_times)
                intensity = np.clip(np.rint(points[:, 4]), 0, 255).astype(np.uint8)
                writer.write(points_optimized, intensity)

                frame_count += 1
                raw_point_count += len(message.points)
                written += len(points)
                first_time = frame_time if first_time is None else first_time
                last_time = float(point_times[-1])
                if frame_count % 100 == 0:
                    print(
                        f"geometry_frames_written: {frame_count}, points_written: {written}",
                        flush=True,
                    )

    report = {
        "bag_path": str(config["bag"]["path"]),
        "cloud_path": str(output_path),
        "uses_mapping_fusion": False,
        "method": "GNSS_INS_pose_graph_with_livox_imu_scan_deskew",
        "deskew_orientation_source": deskew_source,
        "navigation_time_offset_s": navigation_time_offset,
        "time_alignment_correlation": (
            float(alignment.correlation) if np.isfinite(alignment.correlation) else None
        ),
        "lidar_frame_count": int(frame_count),
        "raw_lidar_point_count": int(raw_point_count),
        "cloud_point_count": int(written),
        "first_lidar_time": first_time,
        "last_lidar_time": last_time,
        "correction_time_start": float(correction_times[0]),
        "correction_time_end": float(correction_times[-1]),
    }
    write_json(output_root / "optimized_cloud_report.json", report)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    report = build_cloud(config)
    for key, value in report.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
