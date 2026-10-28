#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from survey_mapping.io import write_csv, write_json
from survey_mapping.lidar_odometry import register_point_to_plane


def nearest_rmse(source, target, transform, max_distance):
    transformed = (transform[:3, :3] @ source.T).T + transform[:3, 3]
    distances, _indices = cKDTree(target).query(transformed, k=1)
    keep = distances <= float(max_distance)
    if not np.any(keep):
        return float("inf"), 0.0
    return float(np.sqrt(np.mean(distances[keep] ** 2))), float(np.count_nonzero(keep) / len(source))


def load_keyframes(root):
    keyframes = []
    for path in sorted((Path(root) / "keyframes").glob("kf_*.npz")):
        data = np.load(path)
        keyframes.append(
            {
                "path": path,
                "time": float(data["time"]),
                "transform": np.asarray(data["transform_initial"], dtype=float),
                "points": np.asarray(data["points_local"], dtype=float),
            }
        )
    return keyframes


def candidate_pairs(keyframes, config):
    pairs = [(index, index + 1, "adjacent") for index in range(len(keyframes) - 1)]
    registration = config.get("registration", {})
    loop_gap = int(registration.get("strip_min_keyframe_gap", 20))
    loop_radius = float(registration.get("strip_search_radius_m", 25.0))
    max_strip_edges = int(registration.get("max_strip_edges_per_keyframe", 2))
    for second in range(loop_gap, len(keyframes)):
        candidates = []
        position_second = keyframes[second]["transform"][:3, 3]
        for first in range(0, second - loop_gap + 1):
            distance = float(np.linalg.norm(position_second - keyframes[first]["transform"][:3, 3]))
            if distance <= loop_radius:
                candidates.append((distance, first))
        for _distance, first in sorted(candidates)[:max_strip_edges]:
            pairs.append((first, second, "strip"))
    return pairs


def quality_rejection_reasons(
    result,
    correction_translation,
    correction_rotation,
    minimum_inlier,
    maximum_rmse,
    maximum_translation,
    maximum_rotation,
):
    reasons = []
    if result.inlier_fraction < minimum_inlier:
        reasons.append("low_inlier_fraction")
    if result.rmse > maximum_rmse:
        reasons.append("high_point_to_plane_rmse")
    if correction_translation > maximum_translation:
        reasons.append("translation_correction_too_large")
    if correction_rotation > maximum_rotation:
        reasons.append("rotation_correction_too_large")
    return reasons


def quality_limits(registration, edge_type):
    """Keep strip edges strict while permitting separately tuned odometry edges."""
    prefix = "adjacent_" if edge_type == "adjacent" else ""
    return {
        "minimum_inlier": float(
            registration.get(
                f"{prefix}minimum_inlier_fraction",
                registration.get("minimum_inlier_fraction", 0.25),
            )
        ),
        "maximum_rmse": float(
            registration.get(
                f"{prefix}maximum_rmse_m", registration.get("maximum_rmse_m", 0.8)
            )
        ),
        "maximum_translation": float(
            registration.get(
                f"{prefix}maximum_correction_m",
                registration.get("maximum_correction_m", 2.0),
            )
        ),
        "maximum_rotation": float(
            registration.get(
                f"{prefix}maximum_correction_deg",
                registration.get("maximum_correction_deg", 4.0),
            )
        ),
    }


def graph_component_count(vertex_count, edges):
    parents = list(range(vertex_count))

    def find(vertex):
        while parents[vertex] != vertex:
            parents[vertex] = parents[parents[vertex]]
            vertex = parents[vertex]
        return vertex

    for first, second in edges:
        first_root = find(first)
        second_root = find(second)
        if first_root != second_root:
            parents[second_root] = first_root
    return len({find(vertex) for vertex in range(vertex_count)})


def build_constraints(config):
    output_root = Path(config["outputs"]["root"])
    keyframes = load_keyframes(output_root)
    if len(keyframes) < 2:
        raise RuntimeError("At least two keyframes are required")
    registration = config.get("registration", {})
    max_distance = float(registration.get("max_correspondence_m", 2.0))
    iterations = int(registration.get("iterations", 30))
    accepted = []
    rejected = []
    transforms = []
    pairs = candidate_pairs(keyframes, config)
    preserve_adjacent = bool(registration.get("preserve_adjacent_connectivity", True))

    for first, second, edge_type in pairs:
        target = keyframes[first]
        source = keyframes[second]
        initial = np.linalg.inv(target["transform"]) @ source["transform"]
        initial_rmse, initial_inlier = nearest_rmse(source["points"], target["points"], initial, max_distance)
        result = register_point_to_plane(
            source["points"], target["points"], initial, max_distance=max_distance, iterations=iterations
        )
        correction = result.transform @ np.linalg.inv(initial)
        correction_translation = float(np.linalg.norm(correction[:3, 3]))
        correction_rotation = float(np.degrees(Rotation.from_matrix(correction[:3, :3]).magnitude()))
        limits = quality_limits(registration, edge_type)
        reasons = quality_rejection_reasons(
            result,
            correction_translation,
            correction_rotation,
            limits["minimum_inlier"],
            limits["maximum_rmse"],
            limits["maximum_translation"],
            limits["maximum_rotation"],
        )
        row = [first, second, edge_type, initial_rmse, initial_inlier, result.rmse, result.inlier_fraction, correction_translation, correction_rotation, result.iterations]
        if reasons:
            rejected.append(row + [";".join(reasons)])
            if preserve_adjacent and edge_type == "adjacent":
                edge_index = len(transforms)
                transforms.append(initial)
                accepted.append(
                    [
                        first,
                        second,
                        "adjacent_fallback",
                        initial_rmse,
                        initial_inlier,
                        max(float(result.rmse), 1.0),
                        min(float(result.inlier_fraction), 0.25),
                        0.0,
                        0.0,
                        0,
                        edge_index,
                    ]
                )
        else:
            edge_index = len(transforms)
            transforms.append(result.transform)
            accepted.append(row + [edge_index])

    header = ["first", "second", "type", "initial_nn_rmse_m", "initial_inlier_fraction", "point_to_plane_rmse_m", "inlier_fraction", "correction_translation_m", "correction_rotation_deg", "iterations"]
    write_csv(output_root / "lidar_constraints.csv", header + ["transform_index"], accepted)
    write_csv(output_root / "lidar_constraints_rejected.csv", header + ["reason"], rejected)
    np.savez_compressed(output_root / "lidar_constraint_transforms.npz", transforms=np.asarray(transforms))
    component_count = graph_component_count(
        len(keyframes), [(int(row[0]), int(row[1])) for row in accepted]
    )
    report = {
        "keyframe_count": len(keyframes),
        "candidate_count": len(pairs),
        "accepted_count": len(accepted),
        "accepted_icp_count": sum(
            row[2] != "adjacent_fallback" for row in accepted
        ),
        "rejected_count": len(rejected),
        "accepted_adjacent_count": sum(row[2].startswith("adjacent") for row in accepted),
        "adjacent_fallback_count": sum(row[2] == "adjacent_fallback" for row in accepted),
        "rejected_adjacent_count": sum(row[2] == "adjacent" for row in rejected),
        "accepted_strip_count": sum(row[2] == "strip" for row in accepted),
        "rejected_strip_count": sum(row[2] == "strip" for row in rejected),
        "graph_component_count": component_count,
        "accepted_mean_initial_nn_rmse_m": float(
            np.mean([row[3] for row in accepted if row[2] != "adjacent_fallback"])
        ) if any(row[2] != "adjacent_fallback" for row in accepted) else None,
        "accepted_mean_point_to_plane_rmse_m": float(
            np.mean([row[5] for row in accepted if row[2] != "adjacent_fallback"])
        ) if any(row[2] != "adjacent_fallback" for row in accepted) else None,
    }
    write_json(output_root / "lidar_constraint_report.json", report)
    if component_count != 1:
        raise RuntimeError(
            f"Accepted LiDAR constraint graph is disconnected: {component_count} components"
        )
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    report = build_constraints(config)
    for key, value in report.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()

