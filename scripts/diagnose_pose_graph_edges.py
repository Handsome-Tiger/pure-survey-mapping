#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from scripts.build_lidar_constraints import load_keyframes
from scripts.optimize_lidar_pose_graph import load_edges
from survey_mapping.pose_graph import read_g2o_poses


def transform_error(measurement, predicted):
    error = np.linalg.inv(measurement) @ predicted
    return (
        float(np.linalg.norm(error[:3, 3])),
        float(np.degrees(Rotation.from_matrix(error[:3, :3]).magnitude())),
    )


def quantiles(values):
    values = np.asarray(values, dtype=float)
    return np.quantile(values, [0.0, 0.5, 0.9, 0.95, 0.99, 1.0]).tolist()


def summarize(output_root):
    output_root = Path(output_root)
    keyframes = load_keyframes(output_root)
    initial = [item["transform"] for item in keyframes]
    optimized_by_id = read_g2o_poses(
        output_root / "pose_graph_optimized.g2o",
        maximum_vertex_id=len(initial) - 1,
    )
    optimized = [optimized_by_id[index] for index in range(len(initial))]
    edges = load_edges(output_root)
    with (output_root / "lidar_constraints.csv").open(
        "r", encoding="utf-8", newline=""
    ) as handle:
        labels = [row["type"] for row in csv.DictReader(handle)]

    result = {}
    for label in ("all", "adjacent", "strip"):
        before_errors = []
        after_errors = []
        for edge_label, edge in zip(labels, edges):
            if label != "all" and edge_label != label:
                continue
            first, second, measurement = edge[:3]
            before_errors.append(
                transform_error(
                    measurement, np.linalg.inv(initial[first]) @ initial[second]
                )
            )
            after_errors.append(
                transform_error(
                    measurement, np.linalg.inv(optimized[first]) @ optimized[second]
                )
            )
        before_errors = np.asarray(before_errors)
        after_errors = np.asarray(after_errors)
        result[label] = {
            "count": int(len(before_errors)),
            "before_translation_quantiles_m": quantiles(before_errors[:, 0]),
            "after_translation_quantiles_m": quantiles(after_errors[:, 0]),
            "before_rotation_quantiles_deg": quantiles(before_errors[:, 1]),
            "after_rotation_quantiles_deg": quantiles(after_errors[:, 1]),
        }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", required=True)
    args = parser.parse_args()
    for key, value in summarize(args.output_root).items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()

