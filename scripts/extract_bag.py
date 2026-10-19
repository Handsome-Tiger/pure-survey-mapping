#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from survey_mapping.io import write_csv
from survey_mapping.livox import filter_valid_points
from survey_mapping.rosraw import (
    parse_imu,
    parse_livox_custom,
    parse_navsatfix,
    parse_quaternion_stamped,
)


def raw_bytes(raw_msg):
    return raw_msg[1]


def extract(config):
    import rosbag

    bag_path = Path(config["bag"]["path"])
    out_root = Path(config["outputs"]["root"]) / "extracted"
    out_root.mkdir(parents=True, exist_ok=True)

    topics = {
        config["bag"]["gnss_topic"],
        config["bag"]["imu_topic"],
        config["bag"]["attitude_topic"],
        config["bag"]["lidar_topic"],
    }
    gnss_rows = []
    imu_rows = []
    attitude_rows = []
    lidar_frames = []
    max_lidar_frames = int(config["mapping"].get("max_lidar_frames", 1))
    max_points_per_frame = int(config["mapping"].get("max_points_per_frame", 20000))
    start_lidar_frame = int(config["mapping"].get("start_lidar_frame", 0))
    valid_lidar_seen = 0

    with rosbag.Bag(str(bag_path)) as bag:
        for topic, raw_msg, _stamp in bag.read_messages(topics=list(topics), raw=True):
            data = raw_bytes(raw_msg)
            if topic == config["bag"]["gnss_topic"]:
                msg = parse_navsatfix(data)
                gnss_rows.append([msg.stamp, msg.latitude, msg.longitude, msg.altitude, msg.status, msg.service])
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
            elif topic == config["bag"]["lidar_topic"] and len(lidar_frames) < max_lidar_frames:
                msg = parse_livox_custom(data)
                points = filter_valid_points(msg.points)
                if len(points):
                    if valid_lidar_seen >= start_lidar_frame:
                        lidar_frames.append({"stamp": msg.timebase_ns * 1e-9, "points": points[:max_points_per_frame]})
                    valid_lidar_seen += 1

    write_csv(
        out_root / "gnss.csv",
        ["time", "latitude", "longitude", "altitude", "status", "service"],
        gnss_rows,
    )
    write_csv(
        out_root / "imu.csv",
        ["time", "gx", "gy", "gz", "ax", "ay", "az"],
        imu_rows,
    )
    write_csv(
        out_root / "attitude.csv",
        ["time", "qx", "qy", "qz", "qw"],
        attitude_rows,
    )
    np.savez_compressed(
        out_root / "livox_frames.npz",
        stamps=np.asarray([frame["stamp"] for frame in lidar_frames], dtype=float),
        **{f"points_{idx}": frame["points"] for idx, frame in enumerate(lidar_frames)},
    )
    return {
        "gnss": len(gnss_rows),
        "imu": len(imu_rows),
        "attitude": len(attitude_rows),
        "lidar_frames": len(lidar_frames),
        "output": str(out_root),
    }


def main():
    parser = argparse.ArgumentParser(description="Extract survey mapping observations from a ROS1 bag.")
    parser.add_argument("--config", default="config/default.yaml")
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    summary = extract(config)
    for key, value in summary.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
