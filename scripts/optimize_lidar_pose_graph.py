#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from scripts.build_lidar_constraints import load_keyframes
from survey_mapping.io import write_csv, write_json
from survey_mapping.pose_graph import (
    pose_correction_metrics,
    read_g2o_poses,
    run_g2o,
    write_g2o_graph,
)


def load_edges(
    output_root,
    rotation_weight_scale=1.0,
    adjacent_fallback_translation_weight=25.0,
):
    transforms = np.load(Path(output_root) / "lidar_constraint_transforms.npz")[
        "transforms"
    ]
    edges = []
    with (Path(output_root) / "lidar_constraints.csv").open(
        "r", encoding="utf-8", newline=""
    ) as handle:
        for row in csv.DictReader(handle):
            index = int(float(row["transform_index"]))
            if row.get("type") == "adjacent_fallback":
                translation_weight = float(adjacent_fallback_translation_weight)
            else:
                rmse = max(float(row["point_to_plane_rmse_m"]), 0.05)
                inlier = float(row["inlier_fraction"])
                translation_weight = float(np.clip(inlier / (rmse * rmse), 1.0, 100.0))
            rotation_weight = translation_weight * float(rotation_weight_scale)
            edges.append(
                (
                    int(float(row["first"])),
                    int(float(row["second"])),
                    transforms[index],
                    translation_weight,
                    rotation_weight,
                )
            )
    return edges


def optimize(config):
    output_root = Path(config["outputs"]["root"])
    keyframes = load_keyframes(output_root)
    initial = [keyframe["transform"] for keyframe in keyframes]
    optimization = config.get("optimization", {})
    lidar_rotation_scale = float(
        optimization.get("lidar_rotation_weight_scale", 1.0)
    )
    fallback_translation_weight = float(
        optimization.get("adjacent_fallback_translation_weight", 25.0)
    )
    edges = load_edges(
        output_root,
        rotation_weight_scale=lidar_rotation_scale,
        adjacent_fallback_translation_weight=fallback_translation_weight,
    )
    graph_path = output_root / "pose_graph.g2o"
    optimized_path = output_root / "pose_graph_optimized.g2o"
    prior_translation_weight = float(
        optimization.get("rtk_translation_weight", 100.0)
    )
    prior_rotation_weight = float(
        optimization.get("attitude_rotation_weight", 100.0)
    )
    write_g2o_graph(
        initial,
        edges,
        graph_path,
        prior_translation_weight=prior_translation_weight,
        prior_rotation_weight=prior_rotation_weight,
    )
    process = run_g2o(
        graph_path,
        optimized_path,
        iterations=int(optimization.get("iterations", 30)),
        robust_kernel=str(optimization.get("robust_kernel", "Huber")),
        robust_width=float(optimization.get("robust_width", 1.0)),
    )
    optimized_by_id = read_g2o_poses(
        optimized_path, maximum_vertex_id=len(initial) - 1
    )
    optimized = [optimized_by_id[index] for index in range(len(initial))]
    corrections_translation = []
    corrections_rotation = []
    rows = []
    for index, (keyframe, before, after) in enumerate(
        zip(keyframes, initial, optimized)
    ):
        translation, rotation = pose_correction_metrics(before, after)
        corrections_translation.append(translation)
        corrections_rotation.append(rotation)
        quaternion = Rotation.from_matrix(after[:3, :3]).as_quat()
        rows.append(
            [
                index,
                keyframe["time"],
                *after[:3, 3].tolist(),
                *quaternion.tolist(),
                translation,
                rotation,
            ]
        )
    max_translation = float(optimization.get("maximum_pose_correction_m", 3.0))
    max_rotation = float(optimization.get("maximum_pose_correction_deg", 6.0))
    correction_vectors = np.asarray(
        [after[:3, 3] - before[:3, 3] for before, after in zip(initial, optimized)]
    )
    adjacent_translation_steps = np.linalg.norm(
        np.diff(correction_vectors, axis=0), axis=1
    )
    correction_rotations = [
        after[:3, :3] @ before[:3, :3].T
        for before, after in zip(initial, optimized)
    ]
    adjacent_rotation_steps = np.asarray(
        [
            np.degrees(
                Rotation.from_matrix(second @ first.T).magnitude()
            )
            for first, second in zip(correction_rotations, correction_rotations[1:])
        ]
    )
    max_adjacent_translation = float(
        optimization.get("maximum_adjacent_correction_step_m", max_translation)
    )
    max_adjacent_rotation = float(
        optimization.get("maximum_adjacent_correction_step_deg", max_rotation)
    )
    if max(corrections_translation, default=0.0) > max_translation:
        raise RuntimeError("Pose graph translation correction exceeded safety limit")
    if max(corrections_rotation, default=0.0) > max_rotation:
        raise RuntimeError("Pose graph rotation correction exceeded safety limit")
    if max(adjacent_translation_steps, default=0.0) > max_adjacent_translation:
        raise RuntimeError("Pose graph adjacent translation step exceeded safety limit")
    if max(adjacent_rotation_steps, default=0.0) > max_adjacent_rotation:
        raise RuntimeError("Pose graph adjacent rotation step exceeded safety limit")
    np.savez_compressed(
        output_root / "keyframe_poses_optimized.npz",
        transforms=np.asarray(optimized),
    )
    write_csv(
        output_root / "trajectory_optimized_keyframes.csv",
        [
            "keyframe_index",
            "time",
            "x",
            "y",
            "z",
            "qx",
            "qy",
            "qz",
            "qw",
            "correction_translation_m",
            "correction_rotation_deg",
        ],
        rows,
    )
    report = {
        "keyframe_count": len(initial),
        "lidar_edge_count": len(edges),
        "lidar_rotation_weight_scale": lidar_rotation_scale,
        "adjacent_fallback_translation_weight": fallback_translation_weight,
        "prior_translation_weight": prior_translation_weight,
        "prior_rotation_weight": prior_rotation_weight,
        "mean_translation_correction_m": float(np.mean(corrections_translation)),
        "max_translation_correction_m": float(np.max(corrections_translation)),
        "mean_rotation_correction_deg": float(np.mean(corrections_rotation)),
        "max_rotation_correction_deg": float(np.max(corrections_rotation)),
        "max_adjacent_correction_step_m": float(
            np.max(adjacent_translation_steps, initial=0.0)
        ),
        "max_adjacent_correction_step_deg": float(
            np.max(adjacent_rotation_steps, initial=0.0)
        ),
        "g2o_stdout_tail": process.stdout[-2000:],
        "g2o_stderr_tail": process.stderr[-2000:],
    }
    write_json(output_root / "optimization_report.json", report)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    report = optimize(config)
    for key, value in report.items():
        if not key.endswith("tail"):
            print(f"{key}: {value}")


if __name__ == "__main__":
    main()


