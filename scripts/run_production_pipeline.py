#!/usr/bin/env python3
"""Run the validated survey-mapping production stages from one command."""

from __future__ import annotations

import argparse
import copy
import shutil
import subprocess
import sys
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]

REQUIRED_TOPICS = {
    "/livox/lidar": "livox_ros_driver/CustomMsg",
    "/livox/imu": "sensor_msgs/Imu",
    "/dji_osdk_ros/imu": "sensor_msgs/Imu",
    "/dji_osdk_ros/attitude": "geometry_msgs/QuaternionStamped",
    "/dji_osdk_ros/rtk_position": "sensor_msgs/NavSatFix",
    "/dji_osdk_ros/rtk_velocity": "geometry_msgs/Vector3Stamped",
    "/dji_osdk_ros/rtk_yaw": "std_msgs/Int16",
    "/dji_osdk_ros/rtk_info_position": "std_msgs/UInt8",
    "/left_camera/image/compressed": "sensor_msgs/CompressedImage",
}

STAGES = (
    ("tests", None, Path(".pytest_cache")),
    ("keyframes", "build_lidar_keyframes.py", Path("keyframe_report.json")),
    ("time-sync", "analyze_time_sync.py", Path("hardware_time_sync_report.json")),
    ("constraints", "build_lidar_constraints.py", Path("lidar_constraint_report.json")),
    ("optimization", "optimize_lidar_pose_graph.py", Path("optimization_report.json")),
    ("geometry", "build_optimized_cloud.py", Path("optimized_cloud_report.json")),
    ("color", "build_optimized_color_cloud.py", Path("full_color_report.json")),
    ("verify", "verify_color_cloud.py", Path("color_cloud_verification.json")),
)


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    if not isinstance(config, dict):
        raise ValueError("Configuration root must be a YAML mapping")
    return config


def resolved_config(config: dict, bag: Path | None, output: Path | None) -> dict:
    result = copy.deepcopy(config)
    if bag is not None:
        result.setdefault("bag", {})["path"] = str(bag.resolve())
    if output is not None:
        result.setdefault("outputs", {})["root"] = str(output.resolve())
        mapping = result.setdefault("mapping", {})
        camera = result.setdefault("camera", {})
        mapping["optimized_cloud_output"] = str(
            output.resolve() / "cloud_optimized_geometry_livox_deskew.ply"
        )
        camera["complete_output"] = str(
            output.resolve() / "cloud_optimized_color_complete.ply"
        )
        camera["strict_output"] = str(
            output.resolve() / "cloud_optimized_color_strict.ply"
        )
    return result


def configured_topics(config: dict) -> dict[str, str]:
    bag = config.get("bag", {})
    camera = config.get("camera", {})
    keys = {
        "lidar_topic": "livox_ros_driver/CustomMsg",
        "livox_imu_topic": "sensor_msgs/Imu",
        "imu_topic": "sensor_msgs/Imu",
        "attitude_topic": "geometry_msgs/QuaternionStamped",
        "gnss_topic": "sensor_msgs/NavSatFix",
        "rtk_velocity_topic": "geometry_msgs/Vector3Stamped",
        "rtk_yaw_topic": "std_msgs/Int16",
        "rtk_quality_topic": "std_msgs/UInt8",
    }
    missing = [f"bag.{key}" for key in keys if key not in bag]
    if "topic" not in camera:
        missing.append("camera.topic")
    if missing:
        raise ValueError("Missing required topic configuration: " + ", ".join(missing))
    topics = {str(bag[key]): expected for key, expected in keys.items()}
    topics[str(camera["topic"])] = "sensor_msgs/CompressedImage"
    return topics


def rosbag_topics(path: Path) -> dict[str, str]:
    process = subprocess.run(
        ["rosbag", "info", "--yaml", str(path)],
        cwd=PROJECT_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    info = yaml.safe_load(process.stdout)
    return {item["topic"]: item["type"] for item in info.get("topics", [])}


def validate_config(config: dict) -> Path:
    bag_path = Path(config.get("bag", {}).get("path", ""))
    if not bag_path.is_absolute():
        bag_path = PROJECT_ROOT / bag_path
    if not bag_path.is_file():
        raise FileNotFoundError(f"ROS bag not found: {bag_path}")
    mapping = config.get("mapping", {})
    forbidden = {
        "start_lidar_frame": mapping.get("start_lidar_frame", 0),
        "max_lidar_frames": mapping.get("max_lidar_frames"),
        "max_points_per_frame": mapping.get("max_points_per_frame"),
    }
    if int(forbidden["start_lidar_frame"] or 0) != 0:
        raise ValueError("Production run requires mapping.start_lidar_frame=0")
    if forbidden["max_lidar_frames"] is not None or forbidden["max_points_per_frame"] is not None:
        raise ValueError("Production run must not limit LiDAR frames or points")
    if mapping.get("deskew_orientation_source") != "livox_imu":
        raise ValueError("Production run requires mapping.deskew_orientation_source=livox_imu")
    alignment = config.get("time_alignment", {})
    if alignment.get("method") != "gyro_correlation":
        raise ValueError("Production run requires time_alignment.method=gyro_correlation")
    if bool(alignment.get("allow_fixed_fallback", False)):
        raise ValueError("Production run forbids fixed time-alignment fallback")
    camera = config.get("camera", {})
    for key in ("intrinsics", "distortion", "rotation_camera_lidar", "translation_camera_lidar_m"):
        if key not in camera:
            raise ValueError(f"Missing calibrated camera field: camera.{key}")
    if not bool(camera.get("full_bag_required", False)):
        raise ValueError("Production run requires camera.full_bag_required=true")
    if shutil.which("g2o") is None:
        raise RuntimeError("g2o executable is not on PATH")
    actual = rosbag_topics(bag_path)
    problems = []
    for topic, expected_type in configured_topics(config).items():
        actual_type = actual.get(topic)
        if actual_type is None:
            problems.append(f"missing {topic} ({expected_type})")
        elif actual_type != expected_type:
            problems.append(f"{topic}: expected {expected_type}, got {actual_type}")
    if problems:
        raise ValueError("ROS bag topic check failed:\n  - " + "\n  - ".join(problems))
    return bag_path


def command_for(stage: str, script: str | None, config_path: Path) -> list[str]:
    if stage == "tests":
        return [sys.executable, "-m", "pytest", "tests", "-q"]
    return [sys.executable, str(Path("scripts") / str(script)), "--config", str(config_path)]


def run_pipeline(args: argparse.Namespace) -> None:
    source = Path(args.config).resolve()
    config = resolved_config(
        load_config(source),
        Path(args.bag) if args.bag else None,
        Path(args.output) if args.output else None,
    )
    bag_path = validate_config(config)
    output_root = Path(config["outputs"]["root"])
    if not output_root.is_absolute():
        output_root = (PROJECT_ROOT / output_root).resolve()
        config["outputs"]["root"] = str(output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    resolved_path = output_root / "resolved_production_config.yaml"
    with resolved_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(config, handle, allow_unicode=True, sort_keys=False)

    start_index = next(i for i, item in enumerate(STAGES) if item[0] == args.from_step)
    print(f"bag: {bag_path}")
    print(f"output: {output_root}")
    for stage, script, marker in STAGES[start_index:]:
        marker_path = (PROJECT_ROOT / marker) if stage == "tests" else (output_root / marker)
        if args.skip_tests and stage == "tests":
            continue
        if args.resume and marker_path.exists():
            print(f"[skip] {stage}: {marker_path}")
            continue
        command = command_for(stage, script, resolved_path)
        print("[run] " + " ".join(str(value) for value in command), flush=True)
        if not args.dry_run:
            subprocess.run(command, cwd=PROJECT_ROOT, check=True)
    print("Production pipeline completed.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run the complete GNSS/INS/Livox/camera production pipeline."
    )
    parser.add_argument("--config", default="config/production.example.yaml")
    parser.add_argument("--bag", help="Override bag.path from the YAML file")
    parser.add_argument("--output", help="Override every production output path")
    parser.add_argument("--from-step", choices=[item[0] for item in STAGES], default="tests")
    parser.add_argument("--resume", action="store_true", help="Skip stages whose report marker exists")
    parser.add_argument("--skip-tests", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    run_pipeline(parser.parse_args())


if __name__ == "__main__":
    main()

