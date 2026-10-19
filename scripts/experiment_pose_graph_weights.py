#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from scripts.build_lidar_constraints import load_keyframes
from scripts.diagnose_pose_graph_edges import transform_error
from scripts.optimize_lidar_pose_graph import load_edges
from survey_mapping.pose_graph import (
    pose_correction_metrics,
    read_g2o_poses,
    run_g2o,
    write_g2o_graph,
)


ROTATION_DIAGONAL_INDICES = (15, 18, 20)


def scale_lidar_rotation_information(path, scale):
    path = Path(path)
    output = []
    for line in path.read_text(encoding="ascii").splitlines():
        fields = line.split()
        if fields and fields[0] == "EDGE_SE3:QUAT" and int(fields[1]) < 100000:
            for index in ROTATION_DIAGONAL_INDICES:
                field_index = 10 + index
                fields[field_index] = str(
                    float(fields[field_index]) * float(scale)
                )
            line = " ".join(fields)
        output.append(line)
    path.write_text("\n".join(output) + "\n", encoding="ascii")


def evaluate(initial, optimized, edges):
    corrections = np.asarray(
        [
            pose_correction_metrics(before, after)
            for before, after in zip(initial, optimized)
        ]
    )
    edge_errors = []
    for edge in edges:
        first, second, measurement = edge[:3]
        predicted = np.linalg.inv(optimized[first]) @ optimized[second]
        edge_errors.append(transform_error(measurement, predicted))
    edge_errors = np.asarray(edge_errors)
    return {
        "max_translation_correction_m": float(np.max(corrections[:, 0])),
        "p95_translation_correction_m": float(np.quantile(corrections[:, 0], 0.95)),
        "max_rotation_correction_deg": float(np.max(corrections[:, 1])),
        "p95_rotation_correction_deg": float(np.quantile(corrections[:, 1], 0.95)),
        "p95_edge_translation_error_m": float(np.quantile(edge_errors[:, 0], 0.95)),
        "p95_edge_rotation_error_deg": float(np.quantile(edge_errors[:, 1], 0.95)),
    }


def run_variant(output_root, prior_rotation_weight, edge_rotation_scale, robust):
    output_root = Path(output_root)
    initial = [item["transform"] for item in load_keyframes(output_root)]
    edges = load_edges(output_root, rotation_weight_scale=1.0)
    label = (
        f"prior{int(prior_rotation_weight)}_edge{int(edge_rotation_scale)}_"
        f"{'huber' if robust else 'quadratic'}"
    )
    graph = output_root / f"pose_graph_{label}.g2o"
    optimized_path = output_root / f"pose_graph_optimized_{label}.g2o"
    write_g2o_graph(
        initial,
        edges,
        graph,
        prior_translation_weight=100.0,
        prior_rotation_weight=float(prior_rotation_weight),
    )
    scale_lidar_rotation_information(graph, edge_rotation_scale)
    run_g2o(
        graph,
        optimized_path,
        30,
        "Huber" if robust else "",
        1.0,
    )
    poses = read_g2o_poses(optimized_path, maximum_vertex_id=len(initial) - 1)
    optimized = [poses[index] for index in range(len(initial))]
    return label, evaluate(initial, optimized, edges)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--prior-rotation-weight", type=float, required=True)
    parser.add_argument("--edge-rotation-scale", type=float, required=True)
    parser.add_argument("--robust", action="store_true")
    args = parser.parse_args()
    label, result = run_variant(
        args.output_root,
        args.prior_rotation_weight,
        args.edge_rotation_scale,
        args.robust,
    )
    print(f"{label}: {result}")


if __name__ == "__main__":
    main()

