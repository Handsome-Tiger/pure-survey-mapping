#!/usr/bin/env python3
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from survey_mapping.io import write_csv, write_json
from survey_mapping.time_sync_rosraw import (
    fc_fields_to_unix,
    parse_fc_time_utc,
    parse_gps_utc,
    parse_ros_string,
    utc_text_to_unix,
)


def _summary(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return None
    return {
        "minimum_s": float(np.min(values)),
        "median_s": float(np.median(values)),
        "maximum_s": float(np.max(values)),
        "std_s": float(np.std(values)),
    }


def analyze(config):
    import rosbag

    bag_config = config.get("bag", {})
    output_root = Path(config["outputs"]["root"])
    gps_topic = bag_config.get(
        "time_sync_gps_utc_topic", "/dji_osdk_ros/time_sync_gps_utc"
    )
    fc_topic = bag_config.get(
        "time_sync_fc_utc_topic", "/dji_osdk_ros/time_sync_fc_time_utc"
    )
    pps_topic = bag_config.get(
        "time_sync_pps_source_topic", "/dji_osdk_ros/time_sync_pps_source"
    )
    topics = [gps_topic, fc_topic, pps_topic]
    gps_rows = []
    fc_rows = []
    pps_rows = []

    with rosbag.Bag(str(bag_config["path"])) as bag:
        for topic, raw_message, bag_stamp in bag.read_messages(topics=topics, raw=True):
            data = raw_message[1]
            if topic == gps_topic:
                message = parse_gps_utc(data)
                utc_epoch = utc_text_to_unix(message.utc_text)
                gps_rows.append(
                    [
                        message.stamp,
                        utc_epoch,
                        message.stamp - utc_epoch,
                        message.utc_text.strip(),
                    ]
                )
            elif topic == fc_topic:
                message = parse_fc_time_utc(data)
                utc_epoch = fc_fields_to_unix(
                    message.utc_yymmdd, message.utc_hhmmss
                )
                fc_rows.append(
                    [
                        message.stamp,
                        message.fc_timestamp_us,
                        utc_epoch,
                        message.stamp - utc_epoch,
                    ]
                )
            elif topic == pps_topic:
                pps_rows.append([bag_stamp.to_sec(), parse_ros_string(data)])

    write_csv(
        output_root / "time_sync_gps_utc.csv",
        ["message_time", "declared_utc_time", "message_minus_utc_s", "utc_text"],
        gps_rows,
    )
    write_csv(
        output_root / "time_sync_fc_utc.csv",
        ["message_time", "fc_timestamp_us", "declared_utc_time", "message_minus_utc_s"],
        fc_rows,
    )
    write_csv(
        output_root / "time_sync_pps_source.csv",
        ["bag_time", "source"],
        pps_rows,
    )
    pps_counts = Counter(row[1] for row in pps_rows)
    report = {
        "bag_path": str(bag_config["path"]),
        "gps_utc_count": int(len(gps_rows)),
        "fc_utc_count": int(len(fc_rows)),
        "pps_source_count": int(len(pps_rows)),
        "pps_sources": dict(sorted(pps_counts.items())),
        "gps_message_minus_declared_utc": _summary([row[2] for row in gps_rows]),
        "fc_message_minus_declared_utc": _summary([row[3] for row in fc_rows]),
        "interpretation": (
            "PPS/UTC records verify the DJI acquisition clock and PPS source. "
            "The Livox-to-DJI residual offset is estimated separately from dual-IMU motion."
        ),
    }
    alignment_path = output_root / "time_alignment_report.json"
    if alignment_path.exists():
        report["dual_imu_alignment"] = json.loads(alignment_path.read_text(encoding="utf-8"))
    write_json(output_root / "hardware_time_sync_report.json", report)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    report = analyze(config)
    for key, value in report.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
