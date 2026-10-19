#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from scripts.build_lidar_constraints import load_keyframes
from survey_mapping.pose_graph import pose_correction_metrics, read_g2o_poses


def summarize(output_root):
    output_root = Path(output_root)
    keyframes = load_keyframes(output_root)
    initial = [item["transform"] for item in keyframes]
    optimized_by_id = read_g2o_poses(
        output_root / "pose_graph_optimized.g2o",
        maximum_vertex_id=len(initial) - 1,
    )
    optimized = [optimized_by_id[index] for index in range(len(initial))]
    metrics = np.asarray(
        [pose_correction_metrics(before, after) for before, after in zip(initial, optimized)]
    )
    worst_rotation = np.argsort(metrics[:, 1])[-10:][::-1]
    worst_translation = np.argsort(metrics[:, 0])[-10:][::-1]
    with (output_root / "lidar_constraints.csv").open(
        "r", encoding="utf-8", newline=""
    ) as handle:
        rows = list(csv.DictReader(handle))
    type_counts = {}
    for row in rows:
        type_counts[row["type"]] = type_counts.get(row["type"], 0) + 1
    return {
        "translation_quantiles_m": np.quantile(
            metrics[:, 0], [0.0, 0.5, 0.9, 0.95, 0.99, 1.0]
        ).tolist(),
        "rotation_quantiles_deg": np.quantile(
            metrics[:, 1], [0.0, 0.5, 0.9, 0.95, 0.99, 1.0]
        ).tolist(),
        "worst_rotation": [
            {
                "keyframe": int(index),
                "time": float(keyframes[index]["time"]),
                "translation_m": float(metrics[index, 0]),
                "rotation_deg": float(metrics[index, 1]),
            }
            for index in worst_rotation
        ],
        "worst_translation": [
            {
                "keyframe": int(index),
                "time": float(keyframes[index]["time"]),
                "translation_m": float(metrics[index, 0]),
                "rotation_deg": float(metrics[index, 1]),
            }
            for index in worst_translation
        ],
        "constraint_type_counts": type_counts,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", required=True)
    args = parser.parse_args()
    for key, value in summarize(args.output_root).items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()

