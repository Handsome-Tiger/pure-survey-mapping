#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys
import time

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from scripts.build_lidar_keyframes import prepare_deskew
from scripts.build_optimized_cloud import load_optimized_model
from scripts.run_mapping_full_stream import lidar_limits, raw_bytes
from survey_mapping.camera import decode_compressed_bgr
from survey_mapping.color_fusion import (
    CameraView,
    TimeOrderedCameraWindow,
    colorize_optimized_points,
)
from survey_mapping.color_output import DualColorPlyWriter
from survey_mapping.imu_deskew import deskew_points_with_imu
from survey_mapping.io import write_json
from survey_mapping.livox import filter_valid_points
from survey_mapping.rosraw import parse_compressed_image, parse_livox_custom


def color_output_paths(config):
    output_root = Path(config["outputs"]["root"])
    camera = config.get("camera", {})
    complete = Path(
        camera.get(
            "complete_output", output_root / "cloud_optimized_color_complete.ply"
        )
    )
    strict = Path(
        camera.get("strict_output", output_root / "cloud_optimized_color_strict.ply")
    )
    return complete, strict


def validate_color_config(config):
    complete, strict = color_output_paths(config)
    if complete.resolve() == strict.resolve():
        raise ValueError("complete and strict color outputs must be different")

    mapping = config.get("mapping", {})
    if str(mapping.get("deskew_orientation_source", "")).lower() != "livox_imu":
        raise ValueError("color mapping requires deskew_orientation_source: livox_imu")

    camera = config.get("camera", {})
    if bool(camera.get("full_bag_required", False)):
        active_limits = [
            key
            for key in ("start_lidar_frame", "max_lidar_frames", "max_points_per_frame")
            if mapping.get(key) is not None
        ]
        if active_limits:
            joined = ", ".join(active_limits)
            raise ValueError(f"full-bag colorization forbids active frame limits: {joined}")
        if bool(config.get("time_alignment", {}).get("allow_fixed_fallback", False)):
            raise ValueError("full-bag colorization requires measured time alignment")

    if float(camera.get("multi_view_window_s", 0.12)) <= 0.0:
        raise ValueError("camera.multi_view_window_s must be positive")
    if int(camera.get("maximum_camera_candidates", 3)) < 1:
        raise ValueError("camera.maximum_camera_candidates must be positive")
    return complete, strict


def lidar_reference_time(frame_time_s, point_offsets_ns):
    offsets = np.asarray(point_offsets_ns, dtype=float)
    if offsets.size == 0:
        raise ValueError("point_offsets_ns must not be empty")
    return float(frame_time_s) + float(np.max(offsets)) * 1e-9


def corrected_camera_view(
    raw_message,
    camera_time_offset_s,
    navigation_time_offset_s,
    navigation,
    correction,
):
    message = parse_compressed_image(raw_bytes(raw_message))
    camera_livox_time = float(message.stamp) + float(camera_time_offset_s)
    initial_position, initial_quaternion = navigation.sample_many(
        np.array([camera_livox_time + float(navigation_time_offset_s)])
    )
    position, quaternion = correction.correct_poses(
        initial_position,
        initial_quaternion,
        np.array([camera_livox_time]),
    )
    return CameraView(
        time_s=camera_livox_time,
        image_bgr=decode_compressed_bgr(message.data),
        position_world_body=position[0],
        quaternion_world_body=quaternion[0],
    )


def build_color_cloud(config):
    import rosbag

    complete_path, strict_path = validate_color_config(config)
    if not bool(config.get("camera", {}).get("overwrite", False)):
        existing = [path for path in (complete_path, strict_path) if path.exists()]
        if existing:
            raise FileExistsError(f"refusing to overwrite color output: {existing[0]}")

    started = time.monotonic()
    output_root = Path(config["outputs"]["root"])
    output_root.mkdir(parents=True, exist_ok=True)
    navigation, pose_series, correction, correction_times = load_optimized_model(
        output_root
    )
    alignment, imu_rotations = prepare_deskew(config, output_root / "extracted")
    minimum_correlation = float(
        config.get("time_alignment", {}).get("minimum_correlation", 0.8)
    )
    if alignment.used_fallback or not np.isfinite(alignment.correlation):
        raise RuntimeError("camera colorization requires measured dual-IMU time alignment")
    if float(alignment.correlation) < minimum_correlation:
        raise RuntimeError(
            "dual-IMU time alignment correlation is below the configured minimum"
        )

    mapping = config.get("mapping", {})
    camera = config["camera"]
    r_bl = np.asarray(mapping["extrinsic"]["rotation_body_lidar"], dtype=float)
    t_bl = np.asarray(
        mapping["extrinsic"]["translation_body_lidar_m"], dtype=float
    )
    imu_extrinsic = mapping.get("livox_imu_extrinsic", {})
    r_il = np.asarray(
        imu_extrinsic.get("rotation_imu_lidar", np.eye(3)), dtype=float
    )
    t_il = np.asarray(
        imu_extrinsic.get("translation_imu_lidar_m", [0.0, 0.0, 0.0]),
        dtype=float,
    )
    r_cl = np.asarray(camera["rotation_camera_lidar"], dtype=float)
    t_cl = np.asarray(camera["translation_camera_lidar_m"], dtype=float)
    intrinsics = np.asarray(camera["intrinsics"], dtype=float)
    distortion = np.asarray(
        camera.get("distortion", [0.0, 0.0, 0.0, 0.0]), dtype=float
    )
    camera_time_offset = float(camera.get("time_offset_s", 0.0))
    window_s = float(camera.get("multi_view_window_s", 0.12))
    maximum_candidates = int(camera.get("maximum_camera_candidates", 3))
    depth_tolerance = float(camera.get("depth_tolerance_m", 0.3))
    blind = float(mapping.get("blind_m", 0.8))
    navigation_time_offset = float(alignment.offset_s)
    start_frame, maximum_frames, maximum_points = lidar_limits(config)
    lidar_topic = config["bag"]["lidar_topic"]
    camera_topic = camera.get("topic", "/left_camera/image/compressed")

    frame_count = 0
    valid_seen = 0
    raw_point_count = 0
    input_point_count = 0
    projected_observation_count = 0
    visible_observation_count = 0
    paired_frames = 0
    decoded_camera_frames = 0
    nearest_time_differences = []
    first_lidar_time = None
    last_lidar_time = None

    with rosbag.Bag(str(config["bag"]["path"])) as camera_bag, rosbag.Bag(
        str(config["bag"]["path"])
    ) as lidar_bag, DualColorPlyWriter(complete_path, strict_path) as writer:
        camera_iterator = camera_bag.read_messages(topics=[camera_topic], raw=True)

        def read_next_camera():
            nonlocal decoded_camera_frames
            try:
                _topic, raw_camera, _stamp = next(camera_iterator)
            except StopIteration:
                return None
            decoded_camera_frames += 1
            return corrected_camera_view(
                raw_camera,
                camera_time_offset,
                navigation_time_offset,
                navigation,
                correction,
            )

        next_camera = read_next_camera()
        if next_camera is None:
            raise RuntimeError("No camera frames found")
        camera_window = TimeOrderedCameraWindow(
            window_s=window_s, maximum_candidates=maximum_candidates
        )

        for _topic, raw_lidar, _stamp in lidar_bag.read_messages(
            topics=[lidar_topic], raw=True
        ):
            message = parse_livox_custom(raw_bytes(raw_lidar))
            points = filter_valid_points(message.points, min_range=blind)
            if not len(points):
                continue
            if valid_seen < start_frame:
                valid_seen += 1
                continue
            if maximum_frames is not None and frame_count >= maximum_frames:
                break
            valid_seen += 1
            raw_point_count += len(message.points)
            if maximum_points is not None:
                points = points[:maximum_points]

            frame_time = float(message.timebase_ns) * 1e-9
            reference_time = lidar_reference_time(frame_time, points[:, 0])
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
            points_body_reference = (r_bl @ deskewed[:, :3].T).T + t_bl
            reference_times = np.full(len(points), reference_time, dtype=float)
            points_world_initial = navigation.transform_body_points(
                points_body_reference, reference_times + navigation_time_offset
            )
            points_world_optimized = correction.apply(
                points_world_initial, reference_times
            )

            while next_camera is not None and next_camera.time_s <= reference_time + window_s:
                if next_camera.time_s >= reference_time - window_s:
                    camera_window.add(next_camera)
                next_camera = read_next_camera()
            camera_window.discard_before(reference_time - window_s)
            candidates = camera_window.candidates(reference_time)
            if candidates:
                paired_frames += 1
                nearest_time_differences.append(
                    abs(float(candidates[0].time_s) - reference_time)
                )

            intensities = np.clip(np.rint(points[:, 4]), 0, 255).astype(np.uint8)
            rgb, color_valid, statistics = colorize_optimized_points(
                points_world_optimized,
                intensities,
                reference_time,
                candidates,
                r_bl,
                t_bl,
                r_cl,
                t_cl,
                intrinsics,
                distortion,
                depth_tolerance,
            )
            writer.write(points_world_optimized, rgb, color_valid, intensities)

            frame_count += 1
            input_point_count += len(points)
            projected_observation_count += statistics["projected_point_count"]
            visible_observation_count += statistics["visible_point_count"]
            first_lidar_time = (
                frame_time if first_lidar_time is None else first_lidar_time
            )
            last_lidar_time = reference_time
            if frame_count % 100 == 0:
                print(
                    "color_frames: "
                    f"{frame_count}, paired: {paired_frames}, "
                    f"complete_points: {writer.complete_count}, "
                    f"true_color_points: {writer.true_color_count}",
                    flush=True,
                )

    complete_count = int(writer.complete_count)
    strict_count = int(writer.strict_count)
    true_color_count = int(writer.true_color_count)
    true_color_fraction = (
        float(true_color_count / complete_count) if complete_count else 0.0
    )
    paired_fraction = float(paired_frames / frame_count) if frame_count else 0.0
    mean_time_difference = (
        float(np.mean(nearest_time_differences))
        if nearest_time_differences
        else None
    )
    max_time_difference = (
        float(np.max(nearest_time_differences))
        if nearest_time_differences
        else None
    )
    report = {
        "bag_path": str(config["bag"]["path"]),
        "uses_mapping_fusion": False,
        "uses_fast_lio": False,
        "method": "GNSS_INS_pose_graph_livox_imu_deskew_camera_projection_bilinear_zbuffer",
        "deskew_orientation_source": "livox_imu",
        "navigation_time_offset_s": navigation_time_offset,
        "time_alignment_correlation": float(alignment.correlation),
        "time_alignment_method": alignment.method,
        "lidar_frame_count": int(frame_count),
        "paired_camera_frame_count": int(paired_frames),
        "decoded_camera_frame_count": int(decoded_camera_frames),
        "raw_lidar_point_count": int(raw_point_count),
        "input_point_count": int(input_point_count),
        "complete_point_count": complete_count,
        "strict_point_count": strict_count,
        "true_color_point_count": true_color_count,
        "projected_observation_count": int(projected_observation_count),
        "visible_observation_count": int(visible_observation_count),
        "true_color_fraction": true_color_fraction,
        "paired_frame_fraction": paired_fraction,
        "mean_nearest_camera_time_difference_s": mean_time_difference,
        "max_nearest_camera_time_difference_s": max_time_difference,
        "camera_time_offset_s": camera_time_offset,
        "multi_view_window_s": window_s,
        "maximum_camera_candidates": maximum_candidates,
        "complete_cloud_path": str(complete_path),
        "strict_cloud_path": str(strict_path),
        "first_lidar_time": first_lidar_time,
        "last_lidar_time": last_lidar_time,
        "correction_time_start": float(correction_times[0]),
        "correction_time_end": float(correction_times[-1]),
        "elapsed_seconds": float(time.monotonic() - started),
    }
    report["acceptance_gates"] = {
        "complete_count_matches_input": complete_count == input_point_count,
        "strict_count_matches_true_color": strict_count == true_color_count,
        "minimum_true_color_fraction": true_color_fraction
        >= float(camera.get("minimum_true_color_fraction", 0.0)),
        "minimum_paired_frame_fraction": paired_fraction
        >= float(camera.get("minimum_paired_frame_fraction", 0.0)),
        "maximum_mean_camera_time_difference": mean_time_difference is not None
        and mean_time_difference
        <= float(camera.get("maximum_mean_camera_time_difference_s", window_s)),
    }
    write_json(output_root / "full_color_report.json", report)
    write_json(output_root / "optimized_color_report.json", report)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    report = build_color_cloud(config)
    for key, value in report.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()

