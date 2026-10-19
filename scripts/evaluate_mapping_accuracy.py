#!/usr/bin/env python3
import argparse
import json
import math
import os
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


PLY_TYPE_MAP = {
    "char": ("i1", 1),
    "uchar": ("u1", 1),
    "short": ("i2", 2),
    "ushort": ("u2", 2),
    "int": ("i4", 4),
    "uint": ("u4", 4),
    "float": ("f4", 4),
    "double": ("f8", 8),
}

PCD_TYPE_MAP = {
    ("F", 4): "f4",
    ("F", 8): "f8",
    ("U", 1): "u1",
    ("U", 2): "u2",
    ("U", 4): "u4",
    ("I", 1): "i1",
    ("I", 2): "i2",
    ("I", 4): "i4",
}


def read_header(path: Path, end_marker: bytes) -> tuple[list[str], int]:
    lines = []
    offset = 0
    with path.open("rb") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise ValueError(f"Header terminator not found in {path}")
            offset += len(line)
            text = line.decode("ascii", errors="replace").strip()
            lines.append(text)
            if line.strip() == end_marker:
                return lines, offset


def sample_indices(count: int, max_points: int, seed: int) -> np.ndarray:
    if count <= max_points:
        return np.arange(count, dtype=np.int64)
    rng = np.random.default_rng(seed)
    return np.sort(rng.choice(count, size=max_points, replace=False))


def load_ply_points(path: Path, max_points: int, seed: int) -> np.ndarray:
    header, offset = read_header(path, b"end_header")
    if "format binary_little_endian 1.0" not in header:
        raise ValueError(f"Only binary_little_endian PLY is supported: {path}")

    vertex_count = None
    properties = []
    in_vertex = False
    for line in header:
        parts = line.split()
        if len(parts) >= 3 and parts[:2] == ["element", "vertex"]:
            vertex_count = int(parts[2])
            in_vertex = True
            continue
        if len(parts) >= 2 and parts[0] == "element" and parts[1] != "vertex":
            in_vertex = False
        if in_vertex and len(parts) == 3 and parts[0] == "property":
            dtype, _ = PLY_TYPE_MAP[parts[1]]
            properties.append((parts[2], "<" + dtype))

    if vertex_count is None:
        raise ValueError(f"PLY vertex count not found: {path}")
    dtype = np.dtype(properties)
    data = np.memmap(path, dtype=dtype, mode="r", offset=offset, shape=(vertex_count,))
    indices = sample_indices(vertex_count, max_points, seed)
    return np.column_stack([data["x"][indices], data["y"][indices], data["z"][indices]]).astype(np.float64)


def load_pcd_points(path: Path, max_points: int, seed: int) -> np.ndarray:
    header, offset = read_header(path, b"DATA binary")
    metadata = {}
    for line in header:
        parts = line.split()
        if parts:
            metadata[parts[0]] = parts[1:]
    fields = metadata["FIELDS"]
    sizes = list(map(int, metadata["SIZE"]))
    types = metadata["TYPE"]
    counts = list(map(int, metadata.get("COUNT", ["1"] * len(fields))))
    points = int(metadata["POINTS"][0])

    dtype_fields = []
    for field, size, type_code, count in zip(fields, sizes, types, counts):
        dtype = "<" + PCD_TYPE_MAP[(type_code, size)]
        if count == 1:
            dtype_fields.append((field, dtype))
        else:
            dtype_fields.append((field, dtype, (count,)))
    dtype = np.dtype(dtype_fields)
    data = np.memmap(path, dtype=dtype, mode="r", offset=offset, shape=(points,))
    indices = sample_indices(points, max_points, seed)
    return np.column_stack([data["x"][indices], data["y"][indices], data["z"][indices]]).astype(np.float64)


def load_cloud(paths: list[Path], max_points: int, seed: int) -> np.ndarray:
    per_file = max(1, math.ceil(max_points / len(paths)))
    chunks = []
    for index, path in enumerate(paths):
        suffix = path.suffix.lower()
        if suffix == ".ply":
            chunk = load_ply_points(path, per_file, seed + index)
        elif suffix == ".pcd":
            chunk = load_pcd_points(path, per_file, seed + index)
        else:
            raise ValueError(f"Unsupported cloud format: {path}")
        chunks.append(chunk[np.isfinite(chunk).all(axis=1)])
    cloud = np.vstack(chunks)
    if len(cloud) > max_points:
        cloud = cloud[sample_indices(len(cloud), max_points, seed + 999)]
    return cloud


def stats(points: np.ndarray) -> dict:
    return {
        "count": int(len(points)),
        "min": points.min(axis=0).tolist(),
        "max": points.max(axis=0).tolist(),
        "mean": points.mean(axis=0).tolist(),
        "span": (points.max(axis=0) - points.min(axis=0)).tolist(),
    }


def voxel_downsample(points: np.ndarray, voxel: float, max_points: int, seed: int) -> np.ndarray:
    keys = np.floor(points / voxel).astype(np.int64)
    _, unique_indices = np.unique(keys, axis=0, return_index=True)
    down = points[np.sort(unique_indices)]
    if len(down) > max_points:
        down = down[sample_indices(len(down), max_points, seed)]
    return down


def best_fit_transform(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_centroid = source.mean(axis=0)
    target_centroid = target.mean(axis=0)
    centered_source = source - source_centroid
    centered_target = target - target_centroid
    covariance = centered_source.T @ centered_target
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = target_centroid - rotation @ source_centroid
    return rotation, translation


def transform(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return points @ rotation.T + translation


def icp(source: np.ndarray, target: np.ndarray, max_iterations: int, trim_quantile: float) -> dict:
    source_centered = source - source.mean(axis=0)
    target_centered = target - target.mean(axis=0)
    rotation = np.eye(3)
    translation = target.mean(axis=0) - source.mean(axis=0)
    tree = cKDTree(target)

    previous_rmse = None
    history = []
    for iteration in range(max_iterations):
        moved = transform(source, rotation, translation)
        distances, indices = tree.query(moved, k=1, workers=-1)
        threshold = np.quantile(distances, trim_quantile)
        mask = distances <= threshold
        pair_source = source[mask]
        pair_target = target[indices[mask]]
        delta_rotation, delta_translation = best_fit_transform(transform(pair_source, rotation, translation), pair_target)
        rotation = delta_rotation @ rotation
        translation = delta_rotation @ translation + delta_translation
        rmse = float(np.sqrt(np.mean(distances[mask] ** 2)))
        history.append({"iteration": iteration + 1, "rmse": rmse, "pairs": int(mask.sum()), "trim_threshold": float(threshold)})
        if previous_rmse is not None and abs(previous_rmse - rmse) < 1e-4:
            break
        previous_rmse = rmse

    return {"rotation": rotation, "translation": translation, "history": history}


def error_metrics(source_aligned: np.ndarray, target: np.ndarray) -> dict:
    tree = cKDTree(target)
    distances, indices = tree.query(source_aligned, k=1, workers=-1)
    residuals = target[indices] - source_aligned
    horizontal = np.linalg.norm(residuals[:, :2], axis=1)
    vertical = np.abs(residuals[:, 2])
    return {
        "count": int(len(distances)),
        "mean": float(np.mean(distances)),
        "rmse": float(np.sqrt(np.mean(distances ** 2))),
        "median": float(np.median(distances)),
        "p68": float(np.quantile(distances, 0.68)),
        "p90": float(np.quantile(distances, 0.90)),
        "p95": float(np.quantile(distances, 0.95)),
        "p99": float(np.quantile(distances, 0.99)),
        "max": float(np.max(distances)),
        "horizontal_rmse": float(np.sqrt(np.mean(horizontal ** 2))),
        "vertical_rmse": float(np.sqrt(np.mean(vertical ** 2))),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--estimate", required=True, type=Path)
    parser.add_argument("--truth", required=True, nargs="+", type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--sample-estimate", type=int, default=500000)
    parser.add_argument("--sample-truth", type=int, default=500000)
    parser.add_argument("--voxel", type=float, default=1.0)
    parser.add_argument("--icp-points", type=int, default=80000)
    parser.add_argument("--eval-points", type=int, default=200000)
    parser.add_argument("--max-iterations", type=int, default=80)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    estimate_raw = load_cloud([args.estimate], args.sample_estimate, args.seed)
    truth_raw = load_cloud(args.truth, args.sample_truth, args.seed + 100)

    estimate = voxel_downsample(estimate_raw, args.voxel, args.eval_points, args.seed + 200)
    truth = voxel_downsample(truth_raw, args.voxel, args.eval_points, args.seed + 300)
    estimate_icp = estimate[sample_indices(len(estimate), min(args.icp_points, len(estimate)), args.seed + 400)]
    truth_icp = truth[sample_indices(len(truth), min(args.icp_points, len(truth)), args.seed + 500)]

    initial_metrics = error_metrics(estimate, truth)
    result = icp(estimate_icp, truth_icp, max_iterations=args.max_iterations, trim_quantile=0.90)
    aligned = transform(estimate, result["rotation"], result["translation"])
    aligned_metrics = error_metrics(aligned, truth)
    reverse_metrics = error_metrics(truth, aligned)

    report = {
        "estimate_file": str(args.estimate),
        "truth_files": [str(path) for path in args.truth],
        "voxel_m": args.voxel,
        "estimate_stats_sampled": stats(estimate_raw),
        "truth_stats_sampled": stats(truth_raw),
        "estimate_eval_points": int(len(estimate)),
        "truth_eval_points": int(len(truth)),
        "initial_nearest_neighbor_m": initial_metrics,
        "icp": {
            "rotation": result["rotation"].tolist(),
            "translation_m": result["translation"].tolist(),
            "history": result["history"],
        },
        "aligned_nearest_neighbor_m": aligned_metrics,
        "truth_to_aligned_nearest_neighbor_m": reverse_metrics,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report["aligned_nearest_neighbor_m"], ensure_ascii=False, indent=2))
    print(f"Saved: {args.out}")


if __name__ == "__main__":
    main()
