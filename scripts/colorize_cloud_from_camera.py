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
    count_lidar_points,
    extract_navigation,
    lidar_limits,
    raw_bytes,
    run_filter,
    run_filter_error_state,
)
from survey_mapping.camera import (
    colorize_body_points,
    decode_compressed_bgr,
    intrinsics_from_fov,
    project_body_points,
    signed_permutation_rotations,
)
from survey_mapping.geo import quat_to_matrix
from survey_mapping.io import read_numeric_csv, write_csv, write_json
from survey_mapping.livox import filter_valid_points
from survey_mapping.rosraw import parse_compressed_image, parse_livox_custom


def colorized_ply_header(point_count):
    return (
        "ply\n"
        "format ascii 1.0\n"
        f"element vertex {point_count}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    )


def format_colorized_vertex(xyz, color):
    return (
        f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f} "
        f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
    )


def load_or_run_navigation(config):
    extracted_root = extract_navigation(config)
    _, gnss = read_numeric_csv(extracted_root / "gnss.csv")
    _, imu = read_numeric_csv(extracted_root / "imu.csv")
    _, attitude = read_numeric_csv(extracted_root / "attitude.csv")
    if config.get("filter", {}).get("mode", "simple") == "error_state":
        origin, gnss_enu, trajectory, residuals, biases = run_filter_error_state(config, gnss, imu, attitude)
        trajectory_header = ["time", "x", "y", "z", "vx", "vy", "vz", "qx", "qy", "qz", "qw", "res_x", "res_y", "res_z"]
    else:
        origin, gnss_enu, trajectory = run_filter(config, gnss, imu, attitude)
        residuals = np.zeros((0, 6), dtype=float)
        biases = np.zeros((0, 7), dtype=float)
        trajectory_header = ["time", "x", "y", "z", "vx", "vy", "vz", "res_x", "res_y", "res_z"]
    out_root = Path(config["outputs"]["root"])
    out_root.mkdir(parents=True, exist_ok=True)
    write_csv(out_root / "trajectory_fused.csv", trajectory_header, trajectory)
    write_csv(out_root / "trajectory_gnss.csv", ["time", "x", "y", "z"], np.column_stack([gnss[:, 0], gnss_enu]))
    if len(residuals):
        write_csv(out_root / "gnss_update_residual.csv", ["time", "res_x", "res_y", "res_z", "res_3d", "accepted"], residuals)
    if len(biases):
        write_csv(out_root / "imu_bias.csv", ["time", "bg_x", "bg_y", "bg_z", "ba_x", "ba_y", "ba_z"], biases)
    return origin, trajectory, attitude


def camera_intrinsics(config, image_shape):
    camera = config.get("camera", {})
    h, w = image_shape[:2]
    if "intrinsics" in camera:
        return np.asarray(camera["intrinsics"], dtype=float)
    return intrinsics_from_fov(
        w,
        h,
        camera.get("fov_x_deg", 82.7),
        camera.get("fov_y_deg", 66.85),
    )


def body_and_world_points(points, frame_start_time, pose_series, rotation_body_lidar, translation_body_lidar):
    body = (rotation_body_lidar @ points[:, 1:4].T).T + translation_body_lidar
    world = np.zeros((len(points), 3), dtype=float)
    for idx, point in enumerate(points):
        point_time = float(frame_start_time) + float(point[0]) * 1e-9
        pose = pose_series.sample(point_time)
        world[idx] = quat_to_matrix(pose.quaternion) @ body[idx] + pose.position
    return body, world


def select_camera_rotation(config, pose_series, intrinsics, image_shape):
    import rosbag

    camera_cfg = config.get("camera", {})
    if not camera_cfg.get("auto_select_axis", False):
        return np.asarray(camera_cfg.get("rotation_camera_body", np.eye(3)), dtype=float), None

    bag_path = Path(config["bag"]["path"])
    lidar_topic = config["bag"]["lidar_topic"]
    camera_topic = camera_cfg.get("topic", config["bag"].get("camera_topic", "/left_camera/image/compressed"))
    rotation_body_lidar = np.asarray(config["mapping"]["extrinsic"]["rotation_body_lidar"], dtype=float)
    translation_body_lidar = np.asarray(config["mapping"]["extrinsic"]["translation_body_lidar_m"], dtype=float)
    start_lidar_frame, _max_lidar_frames, max_points_per_frame = lidar_limits(config)
    scan_frames = int(camera_cfg.get("axis_select_frames", 40))
    stride = int(camera_cfg.get("axis_select_stride", 80))
    max_dt = float(camera_cfg.get("max_time_offset_s", 0.08))
    latest_image = None
    latest_time = None
    frames = []
    valid_seen = 0
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, raw_msg, _stamp in bag.read_messages(topics=[camera_topic, lidar_topic], raw=True):
            if topic == camera_topic:
                image_msg = parse_compressed_image(raw_bytes(raw_msg))
                latest_image = decode_compressed_bgr(image_msg.data)
                latest_time = image_msg.stamp
                continue
            msg = parse_livox_custom(raw_bytes(raw_msg))
            points = filter_valid_points(msg.points)
            if not len(points):
                continue
            if valid_seen < start_lidar_frame:
                valid_seen += 1
                continue
            valid_seen += 1
            if latest_image is None or abs((msg.timebase_ns * 1e-9) - latest_time) > max_dt:
                continue
            if max_points_per_frame is not None:
                points = points[:max_points_per_frame]
            sample = points[::stride]
            body, _world = body_and_world_points(
                sample, msg.timebase_ns * 1e-9, pose_series, rotation_body_lidar, translation_body_lidar
            )
            frames.append((body, latest_image.shape))
            if len(frames) >= scan_frames:
                break

    rotations = signed_permutation_rotations()
    best = None
    for idx, rotation in enumerate(rotations):
        projected = 0
        total = 0
        for body, shape in frames:
            _uv, valid = project_body_points(body, rotation, np.zeros(3), intrinsics, shape)
            projected += int(np.count_nonzero(valid))
            total += len(body)
        fraction = projected / total if total else 0.0
        score = (fraction, projected, -idx)
        if best is None or score > best[0]:
            best = (score, idx, rotation, projected, total)
    if best is None:
        return np.eye(3), {"axis_select_projected": 0, "axis_select_total": 0}
    _score, idx, rotation, projected, total = best
    return rotation, {
        "axis_select_index": int(idx),
        "axis_select_projected": int(projected),
        "axis_select_total": int(total),
        "axis_select_fraction": float(projected / total) if total else 0.0,
        "rotation_camera_body": rotation.tolist(),
    }


def write_colorized_cloud(config, pose_series, rotation_camera_body, intrinsics, output_path, point_count):
    import rosbag

    bag_path = Path(config["bag"]["path"])
    lidar_topic = config["bag"]["lidar_topic"]
    camera_cfg = config.get("camera", {})
    camera_topic = camera_cfg.get("topic", config["bag"].get("camera_topic", "/left_camera/image/compressed"))
    rotation_body_lidar = np.asarray(config["mapping"]["extrinsic"]["rotation_body_lidar"], dtype=float)
    translation_body_lidar = np.asarray(config["mapping"]["extrinsic"]["translation_body_lidar_m"], dtype=float)
    translation_body_camera = np.asarray(camera_cfg.get("translation_body_camera_m", [0.0, 0.0, 0.0]), dtype=float)
    start_lidar_frame, max_lidar_frames, max_points_per_frame = lidar_limits(config)
    max_dt = float(camera_cfg.get("max_time_offset_s", 0.08))

    valid_seen = 0
    frame_count = 0
    lidar_frames_seen = 0
    written = 0
    colored = 0
    camera_used = 0
    previous_image = None
    previous_time = None
    pending_lidar = []

    def choose_image(frame_time, next_image=None, next_time=None):
        candidates = []
        if previous_image is not None:
            candidates.append((abs(frame_time - previous_time), previous_image, previous_time))
        if next_image is not None:
            candidates.append((abs(frame_time - next_time), next_image, next_time))
        if not candidates:
            return None, None
        dt, image, image_time = min(candidates, key=lambda item: item[0])
        if dt <= max_dt:
            return image, image_time
        return None, None

    def write_one(handle, frame_time, points, image, image_time):
        nonlocal frame_count, written, colored, camera_used
        body, world = body_and_world_points(
            points, frame_time, pose_series, rotation_body_lidar, translation_body_lidar
        )
        intensities = points[:, 4]
        if image is not None:
            rgb, valid = colorize_body_points(
                body, intensities, image, rotation_camera_body, translation_body_camera, intrinsics
            )
            colored += int(np.count_nonzero(valid))
            camera_used += 1
        else:
            gray = np.clip(np.rint(intensities), 0, 255).astype(np.uint8)
            rgb = np.repeat(gray[:, None], 3, axis=1)
        for xyz, color in zip(world, rgb):
            handle.write(format_colorized_vertex(xyz, color))
        frame_count += 1
        written += len(points)
        if frame_count % 250 == 0:
            print(
                f"color_frames_written: {frame_count}, points_written: {written}, "
                f"colored_points: {colored}"
            )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write(colorized_ply_header(point_count))
        with rosbag.Bag(str(bag_path)) as bag:
            for topic, raw_msg, _stamp in bag.read_messages(topics=[camera_topic, lidar_topic], raw=True):
                if topic == camera_topic:
                    image_msg = parse_compressed_image(raw_bytes(raw_msg))
                    current_image = decode_compressed_bgr(image_msg.data)
                    current_time = image_msg.stamp
                    remaining = []
                    for frame_time, points in pending_lidar:
                        if frame_time <= current_time or current_time - frame_time > max_dt:
                            image, image_time = choose_image(frame_time, current_image, current_time)
                            write_one(handle, frame_time, points, image, image_time)
                        else:
                            remaining.append((frame_time, points))
                    pending_lidar = remaining
                    previous_image = current_image
                    previous_time = current_time
                    if max_lidar_frames is not None and frame_count >= max_lidar_frames:
                        break
                    continue

                msg = parse_livox_custom(raw_bytes(raw_msg))
                points = filter_valid_points(msg.points)
                if not len(points):
                    continue
                if valid_seen < start_lidar_frame:
                    valid_seen += 1
                    continue
                if max_lidar_frames is not None and lidar_frames_seen >= max_lidar_frames:
                    break
                valid_seen += 1
                lidar_frames_seen += 1
                if max_points_per_frame is not None:
                    points = points[:max_points_per_frame]
                pending_lidar.append((msg.timebase_ns * 1e-9, points))

        for frame_time, points in pending_lidar:
            image, image_time = choose_image(frame_time)
            write_one(handle, frame_time, points, image, image_time)

    return {
        "cloud_colorized_path": str(output_path),
        "uses_mapping_fusion": False,
        "colorization_method": "lidar_to_body_world_transform_body_to_camera_projection_bilinear_image_sampling",
        "colorization_source": "survey_mapping_bag_lidar_camera",
        "lidar_frame_count": int(frame_count),
        "cloud_point_count": int(written),
        "camera_frames_used": int(camera_used),
        "colored_point_count": int(colored),
        "colored_fraction": float(colored / written) if written else 0.0,
    }

def run(config):
    origin, trajectory, attitude = load_or_run_navigation(config)
    pose_series = build_mapping_pose_series(config, trajectory, attitude)
    lidar_frame_count, lidar_point_count, first_lidar_stamp, last_lidar_stamp = count_lidar_points(config)

    camera_cfg = config.get("camera", {})
    output_path = Path(camera_cfg.get("output", Path(config["outputs"]["root"]) / "cloud_colorized.ply"))

    first_image = None
    import rosbag
    with rosbag.Bag(str(config["bag"]["path"])) as bag:
        camera_topic = camera_cfg.get("topic", config["bag"].get("camera_topic", "/left_camera/image/compressed"))
        for _topic, raw_msg, _stamp in bag.read_messages(topics=[camera_topic], raw=True):
            first_image = decode_compressed_bgr(parse_compressed_image(raw_bytes(raw_msg)).data)
            break
    if first_image is None:
        raise RuntimeError("No camera images found for colorization")

    intrinsics = camera_intrinsics(config, first_image.shape)
    rotation_camera_body, axis_report = select_camera_rotation(config, pose_series, intrinsics, first_image.shape)
    result = write_colorized_cloud(
        config, pose_series, rotation_camera_body, intrinsics, output_path, lidar_point_count
    )
    result.update({
        "origin_lat_lon_h": [float(v) for v in origin],
        "input_lidar_frame_count": int(lidar_frame_count),
        "first_lidar_stamp": float(first_lidar_stamp) if first_lidar_stamp is not None else None,
        "last_lidar_stamp": float(last_lidar_stamp) if last_lidar_stamp is not None else None,
        "camera_topic": camera_cfg.get("topic", config["bag"].get("camera_topic", "/left_camera/image/compressed")),
        "camera_intrinsics": intrinsics.tolist(),
        "rotation_camera_body": rotation_camera_body.tolist(),
        "translation_body_camera_m": camera_cfg.get("translation_body_camera_m", [0.0, 0.0, 0.0]),
    })
    if axis_report:
        result.update(axis_report)
    if result["cloud_point_count"] != lidar_point_count:
        raise RuntimeError("Written colorized point count does not match LiDAR counting pass")
    write_json(Path(config["outputs"]["root"]) / "colorize_report.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description="Project compressed camera images onto the mapped LiDAR cloud.")
    parser.add_argument("--config", default="config/HKisland_GNSS02_kfgins_sample.yaml")
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    report = run(config)
    for key, value in report.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()

