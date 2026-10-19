#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from survey_mapping.geo import geodetic_to_enu, quat_to_matrix
from survey_mapping.io import read_numeric_csv, write_csv, write_json
from survey_mapping.livox import filter_valid_points, georeference_points
from survey_mapping.rosraw import (
    parse_imu,
    parse_int16,
    parse_livox_custom,
    parse_navsatfix,
    parse_uint8,
    parse_vector3_stamped,
    parse_quaternion_stamped,
)
from survey_mapping.trajectory import build_pose_series
from survey_mapping.ekf import GnssInsFilter
from survey_mapping.error_state_ins import ErrorStateConfig, ErrorStateInsFilter


def raw_bytes(raw_msg):
    return raw_msg[1]


def extract_navigation(config):
    import rosbag

    bag_path = Path(config["bag"]["path"])
    out_root = Path(config["outputs"]["root"]) / "extracted"
    out_root.mkdir(parents=True, exist_ok=True)
    topics = [
        config["bag"]["gnss_topic"],
        config["bag"]["imu_topic"],
        config["bag"]["attitude_topic"],
        config["bag"].get("rtk_velocity_topic", "/dji_osdk_ros/rtk_velocity"),
        config["bag"].get("rtk_yaw_topic", "/dji_osdk_ros/rtk_yaw"),
        config["bag"].get("rtk_quality_topic", "/dji_osdk_ros/rtk_info_position"),
        config["bag"].get("livox_imu_topic", "/livox/imu"),
    ]
    gnss_rows = []
    imu_rows = []
    attitude_rows = []
    rtk_velocity_rows = []
    rtk_yaw_rows = []
    rtk_quality_rows = []
    livox_imu_rows = []
    rtk_velocity_topic = config["bag"].get("rtk_velocity_topic", "/dji_osdk_ros/rtk_velocity")
    rtk_yaw_topic = config["bag"].get("rtk_yaw_topic", "/dji_osdk_ros/rtk_yaw")
    rtk_quality_topic = config["bag"].get("rtk_quality_topic", "/dji_osdk_ros/rtk_info_position")
    livox_imu_topic = config["bag"].get("livox_imu_topic", "/livox/imu")
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, raw_msg, _stamp in bag.read_messages(topics=topics, raw=True):
            data = raw_bytes(raw_msg)
            if topic == config["bag"]["gnss_topic"]:
                msg = parse_navsatfix(data)
                gnss_rows.append(
                    [msg.stamp, msg.latitude, msg.longitude, msg.altitude, msg.status, msg.service]
                )
            elif topic == config["bag"]["imu_topic"]:
                msg = parse_imu(data)
                imu_rows.append(
                    [
                        msg.stamp,
                        *msg.angular_velocity.tolist(),
                        *msg.linear_acceleration.tolist(),
                    ]
                )
            elif topic == config["bag"]["attitude_topic"]:
                msg = parse_quaternion_stamped(data)
                attitude_rows.append([msg.stamp, *msg.quaternion.tolist()])
            elif topic == rtk_velocity_topic:
                msg = parse_vector3_stamped(data)
                rtk_velocity_rows.append([msg.stamp, *msg.vector.tolist()])
            elif topic == rtk_yaw_topic:
                rtk_yaw_rows.append([_stamp.to_sec(), parse_int16(data)])
            elif topic == rtk_quality_topic:
                rtk_quality_rows.append([_stamp.to_sec(), parse_uint8(data)])
            elif topic == livox_imu_topic:
                msg = parse_imu(data)
                livox_imu_rows.append([msg.stamp, *msg.angular_velocity.tolist(), *msg.linear_acceleration.tolist()])

    write_csv(
        out_root / "gnss.csv",
        ["time", "latitude", "longitude", "altitude", "status", "service"],
        gnss_rows,
    )
    write_csv(out_root / "imu.csv", ["time", "gx", "gy", "gz", "ax", "ay", "az"], imu_rows)
    write_csv(out_root / "attitude.csv", ["time", "qx", "qy", "qz", "qw"], attitude_rows)
    write_csv(out_root / "rtk_velocity.csv", ["time", "vx", "vy", "vz"], rtk_velocity_rows)
    write_csv(out_root / "rtk_yaw.csv", ["time", "yaw_raw_centideg"], rtk_yaw_rows)
    write_csv(out_root / "rtk_quality.csv", ["time", "position_quality"], rtk_quality_rows)
    write_csv(out_root / "livox_imu.csv", ["time", "gx", "gy", "gz", "ax", "ay", "az"], livox_imu_rows)
    return out_root


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
    for t, accel_body in zip(imu_times, imu_accels_body):
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
    return origin, gnss_enu, np.asarray(trajectory_rows, dtype=float)



def run_filter_error_state(config, gnss, imu, attitude, rtk_velocity=None, rtk_yaw=None, rtk_quality=None):
    origin = tuple(gnss[0, 1:4])
    gnss_times = gnss[:, 0]
    gnss_enu = np.asarray(
        [geodetic_to_enu(lat, lon, alt, origin) for lat, lon, alt in gnss[:, 1:4]],
        dtype=float,
    )
    imu_times = imu[:, 0]
    gyro_body = imu[:, 1:4]
    accel_body = imu[:, 4:7]
    attitude_times = attitude[:, 0] if len(attitude) else None
    quats = attitude[:, 1:5] if len(attitude) else None
    rtk_velocity = np.zeros((0, 4), dtype=float) if rtk_velocity is None else np.asarray(rtk_velocity, dtype=float)
    rtk_yaw = np.zeros((0, 2), dtype=float) if rtk_yaw is None else np.asarray(rtk_yaw, dtype=float)
    rtk_quality = np.zeros((0, 2), dtype=float) if rtk_quality is None else np.asarray(rtk_quality, dtype=float)
    filter_cfg = config.get("filter", {})
    lever_arm = np.asarray(filter_cfg.get("lever_arm_body_m", [0.0, 0.0, 0.0]), dtype=float)
    cfg = ErrorStateConfig(
        accel_noise=float(filter_cfg.get("accel_noise", 0.15)),
        gyro_noise=float(filter_cfg.get("gyro_noise", 0.02)),
        accel_bias_noise=float(filter_cfg.get("accel_bias_noise", 1e-4)),
        gyro_bias_noise=float(filter_cfg.get("gyro_bias_noise", 1e-5)),
        gnss_sigma=float(filter_cfg.get("gnss_sigma_m", filter_cfg.get("fixed_rtk_sigma_m", 0.05))),
        outlier_threshold=float(filter_cfg.get("outlier_threshold_m", 20.0)),
        robust_threshold=float(filter_cfg.get("robust_threshold_m", 4.0)),
        robust_sigma_scale=float(filter_cfg.get("robust_sigma_scale", 5.0)),
        lever_arm_body=lever_arm,
    )
    init_quat = quats[0] if quats is not None and len(quats) else np.array([0.0, 0.0, 0.0, 1.0])
    filt = ErrorStateInsFilter(cfg)
    filt.reset(gnss_enu[0], np.zeros(3), init_quat)
    trajectory_rows = []
    residual_rows = []
    bias_rows = []
    gnss_idx = 0
    velocity_idx = 0
    yaw_idx = 0
    last_t = float(gnss_times[0])
    use_external_attitude = bool(filter_cfg.get("use_external_attitude", True))
    for t, gyro, accel in zip(imu_times, gyro_body, accel_body):
        if t < gnss_times[0]:
            continue
        if use_external_attitude and quats is not None:
            q = quats[min(np.searchsorted(attitude_times, t), len(quats) - 1)]
            filt.quaternion = q / np.linalg.norm(q)
        propagation_gyro = filt.gyro_bias if use_external_attitude and quats is not None else gyro
        filt.propagate(float(t) - last_t, propagation_gyro, accel)
        last_t = float(t)
        residual = np.array([np.nan, np.nan, np.nan])
        accepted_value = np.nan
        while gnss_idx < len(gnss_times) and gnss_times[gnss_idx] <= t:
            obs_sigma = float(filter_cfg.get("fixed_rtk_sigma_m", 0.05))
            if len(rtk_quality):
                quality_idx = min(int(np.searchsorted(rtk_quality[:, 0], gnss_times[gnss_idx])), len(rtk_quality) - 1)
                if int(rtk_quality[quality_idx, 1]) != int(filter_cfg.get("fixed_rtk_quality_value", 50)):
                    obs_sigma = float(filter_cfg.get("float_rtk_sigma_m", 1.5))
            accepted, residual = filt.update_gnss(gnss_enu[gnss_idx], sigma=obs_sigma)
            accepted_value = 1.0 if accepted else 0.0
            residual_rows.append([gnss_times[gnss_idx], *residual.tolist(), float(np.linalg.norm(residual)), accepted_value])
            gnss_idx += 1
        while velocity_idx < len(rtk_velocity) and rtk_velocity[velocity_idx, 0] <= t:
            velocity_ned = rtk_velocity[velocity_idx, 1:4]
            velocity_enu = np.array([velocity_ned[1], velocity_ned[0], -velocity_ned[2]], dtype=float)
            filt.update_velocity(velocity_enu, sigma=float(filter_cfg.get("rtk_velocity_sigma_mps", 0.1)))
            velocity_idx += 1
        while yaw_idx < len(rtk_yaw) and rtk_yaw[yaw_idx, 0] <= t:
            if not use_external_attitude:
                yaw_enu = -np.deg2rad(rtk_yaw[yaw_idx, 1])
                filt.update_yaw(yaw_enu, sigma_rad=np.deg2rad(float(filter_cfg.get("rtk_yaw_sigma_deg", 1.0))))
            yaw_idx += 1
        state = filt.state()
        trajectory_rows.append([t, *state.position.tolist(), *state.velocity.tolist(), *state.quaternion.tolist(), *residual.tolist()])
        bias_rows.append([t, *state.gyro_bias.tolist(), *state.accel_bias.tolist()])
    return origin, gnss_enu, np.asarray(trajectory_rows, dtype=float), np.asarray(residual_rows, dtype=float), np.asarray(bias_rows, dtype=float)

def lidar_limits(config):
    mapping = config.get("mapping", {})
    start = int(mapping.get("start_lidar_frame", 0))
    max_frames = mapping.get("max_lidar_frames")
    max_frames = None if max_frames is None else int(max_frames)
    max_points = mapping.get("max_points_per_frame")
    max_points = None if max_points is None else int(max_points)
    return start, max_frames, max_points
def count_lidar_points(config):
    import rosbag

    bag_path = Path(config["bag"]["path"])
    lidar_topic = config["bag"]["lidar_topic"]
    frame_count = 0
    point_count = 0
    valid_seen = 0
    first_stamp = None
    last_stamp = None
    start_lidar_frame, max_lidar_frames, max_points_per_frame = lidar_limits(config)
    with rosbag.Bag(str(bag_path)) as bag:
        for _topic, raw_msg, _stamp in bag.read_messages(topics=[lidar_topic], raw=True):
            msg = parse_livox_custom(raw_bytes(raw_msg))
            points = filter_valid_points(msg.points)
            if not len(points):
                continue
            if valid_seen < start_lidar_frame:
                valid_seen += 1
                continue
            if max_lidar_frames is not None and frame_count >= max_lidar_frames:
                break
            valid_seen += 1
            if max_points_per_frame is not None:
                points = points[:max_points_per_frame]
            frame_count += 1
            point_count += len(points)
            stamp = msg.timebase_ns * 1e-9
            first_stamp = stamp if first_stamp is None else first_stamp
            last_stamp = stamp
    return frame_count, point_count, first_stamp, last_stamp


def write_lidar_ply_stream(config, pose_series, point_count):
    import rosbag

    bag_path = Path(config["bag"]["path"])
    lidar_topic = config["bag"]["lidar_topic"]
    out_root = Path(config["outputs"]["root"])
    cloud_path = out_root / "cloud_full.ply"
    rotation = np.asarray(config["mapping"]["extrinsic"]["rotation_body_lidar"], dtype=float)
    translation = np.asarray(config["mapping"]["extrinsic"]["translation_body_lidar_m"], dtype=float)
    frame_count = 0
    written = 0
    valid_seen = 0
    start_lidar_frame, max_lidar_frames, max_points_per_frame = lidar_limits(config)
    with cloud_path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("ply\n")
        handle.write("format ascii 1.0\n")
        handle.write(f"element vertex {point_count}\n")
        handle.write("property float x\n")
        handle.write("property float y\n")
        handle.write("property float z\n")
        handle.write("property uchar intensity\n")
        handle.write("end_header\n")
        with rosbag.Bag(str(bag_path)) as bag:
            for _topic, raw_msg, _stamp in bag.read_messages(topics=[lidar_topic], raw=True):
                msg = parse_livox_custom(raw_bytes(raw_msg))
                points = filter_valid_points(msg.points)
                if not len(points):
                    continue
                if valid_seen < start_lidar_frame:
                    valid_seen += 1
                    continue
                if max_lidar_frames is not None and frame_count >= max_lidar_frames:
                    break
                valid_seen += 1
                if max_points_per_frame is not None:
                    points = points[:max_points_per_frame]
                mapped = georeference_points(
                    points, msg.timebase_ns * 1e-9, pose_series, rotation, translation
                )
                for x, y, z, intensity in mapped:
                    clipped = int(max(0, min(255, round(float(intensity)))))
                    handle.write(f"{x:.6f} {y:.6f} {z:.6f} {clipped}\n")
                frame_count += 1
                written += len(mapped)
                if frame_count % 250 == 0:
                    print(f"lidar_frames_written: {frame_count}, points_written: {written}")
    return cloud_path, frame_count, written



def build_mapping_pose_series(config, trajectory, attitude):
    mapping_cfg = config.get("mapping", {})
    source = str(mapping_cfg.get("orientation_source", "attitude")).lower()
    has_filter_quat = trajectory.shape[1] >= 11
    if source == "filter":
        if not has_filter_quat:
            raise ValueError("mapping.orientation_source=filter requires quaternion columns in trajectory")
        return build_pose_series(trajectory[:, 0], trajectory[:, 1:4], trajectory[:, 0], trajectory[:, 7:11])
    if source != "attitude":
        raise ValueError("mapping.orientation_source must be either 'attitude' or 'filter'")
    return build_pose_series(trajectory[:, 0], trajectory[:, 1:4], attitude[:, 0], attitude[:, 1:5])
def run(config):
    extracted_root = extract_navigation(config)
    _, gnss = read_numeric_csv(extracted_root / "gnss.csv")
    _, imu = read_numeric_csv(extracted_root / "imu.csv")
    _, attitude = read_numeric_csv(extracted_root / "attitude.csv")
    _, rtk_velocity = read_numeric_csv(extracted_root / "rtk_velocity.csv")
    _, rtk_yaw = read_numeric_csv(extracted_root / "rtk_yaw.csv")
    _, rtk_quality = read_numeric_csv(extracted_root / "rtk_quality.csv")
    if config.get("filter", {}).get("mode", "simple") == "error_state":
        origin, gnss_enu, trajectory, residuals, biases = run_filter_error_state(
            config, gnss, imu, attitude, rtk_velocity, rtk_yaw, rtk_quality
        )
        trajectory_header = ["time", "x", "y", "z", "vx", "vy", "vz", "qx", "qy", "qz", "qw", "res_x", "res_y", "res_z"]
    else:
        origin, gnss_enu, trajectory = run_filter(config, gnss, imu, attitude)
        residuals = np.zeros((0, 6), dtype=float)
        biases = np.zeros((0, 7), dtype=float)
        trajectory_header = ["time", "x", "y", "z", "vx", "vy", "vz", "res_x", "res_y", "res_z"]
    out_root = Path(config["outputs"]["root"])
    out_root.mkdir(parents=True, exist_ok=True)
    write_csv(
        out_root / "trajectory_fused.csv",
        trajectory_header,
        trajectory,
    )
    write_csv(out_root / "trajectory_gnss.csv", ["time", "x", "y", "z"], np.column_stack([gnss[:, 0], gnss_enu]))
    if len(residuals):
        write_csv(out_root / "gnss_update_residual.csv", ["time", "res_x", "res_y", "res_z", "res_3d", "accepted"], residuals)
    if len(biases):
        write_csv(out_root / "imu_bias.csv", ["time", "bg_x", "bg_y", "bg_z", "ba_x", "ba_y", "ba_z"], biases)
    pose_series = build_mapping_pose_series(config, trajectory, attitude)

    lidar_frame_count, lidar_point_count, first_lidar_stamp, last_lidar_stamp = count_lidar_points(config)
    cloud_path, frames_written, points_written = write_lidar_ply_stream(
        config, pose_series, lidar_point_count
    )
    report = {
        "bag_path": str(config["bag"]["path"]),
        "origin_lat_lon_h": [float(v) for v in origin],
        "gnss_count": int(len(gnss)),
        "imu_count": int(len(imu)),
        "attitude_count": int(len(attitude)),
        "trajectory_count": int(len(trajectory)),
        "filter_mode": str(config.get("filter", {}).get("mode", "simple")),
        "mapping_orientation_source": str(config.get("mapping", {}).get("orientation_source", "attitude")),
        "rotation_body_lidar": config.get("mapping", {}).get("extrinsic", {}).get("rotation_body_lidar"),
        "translation_body_lidar_m": config.get("mapping", {}).get("extrinsic", {}).get("translation_body_lidar_m"),
        "gnss_update_count": int(len(residuals)),
        "gnss_update_accepted": int(np.nansum(residuals[:, 5])) if len(residuals) else 0,
        "lidar_frame_count": int(lidar_frame_count),
        "start_lidar_frame": int(config.get("mapping", {}).get("start_lidar_frame", 0)),
        "max_lidar_frames": config.get("mapping", {}).get("max_lidar_frames"),
        "max_points_per_frame": config.get("mapping", {}).get("max_points_per_frame"),
        "cloud_point_count": int(points_written),
        "cloud_path": str(cloud_path),
        "first_lidar_stamp": float(first_lidar_stamp) if first_lidar_stamp is not None else None,
        "last_lidar_stamp": float(last_lidar_stamp) if last_lidar_stamp is not None else None,
    }
    if points_written != lidar_point_count or frames_written != lidar_frame_count:
        raise RuntimeError("Written LiDAR counts do not match the counting pass")
    write_json(out_root / "report.json", report)
    return report


def main():
    parser = argparse.ArgumentParser(description="Run full-bag stream mapping without LiDAR frame limits.")
    parser.add_argument("--config", default="config/HKisland_GNSS02_full_all.yaml")
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    report = run(config)
    for key, value in report.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
