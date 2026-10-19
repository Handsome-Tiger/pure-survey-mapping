#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Iterable, List, Tuple

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

REPO = Path("/home/shuai/code/mapping-fusion-main")
FAST_WS = Path("/home/shuai/code/fast-livo2-humble-ws")
CORRECT_PCD = REPO / "data/HK_PCD_HKISLAND_COLORIZED_GEOM"
CORRECT_POSE = REPO / "data/HK_POSE_HKISLAND_COLORIZED_GEOM"
RESULTS = REPO / "results_hkisland_blocks_colorized_geom"
COLOR_ROOT = FAST_WS / "runtime/hkisland_blocks_livo_color_recovery"


def natural_key(path: Path):
    try:
        return int(path.stem)
    except ValueError:
        return path.stem


def load_pose_xyz(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    data = np.loadtxt(str(path))
    if data.ndim == 1:
        data = data.reshape(1, -1)
    order = np.argsort(data[:, 0])
    data = data[order]
    ts, idx = np.unique(data[:, 0], return_index=True)
    return ts.astype(float), data[idx, 1:4].astype(float)


def interp_xyz(target_ts: np.ndarray, src_ts: np.ndarray, src_xyz: np.ndarray) -> np.ndarray:
    return np.column_stack([np.interp(target_ts, src_ts, src_xyz[:, i]) for i in range(3)])


def make_T(rot: np.ndarray, trans: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = trans
    return T


def kabsch(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    cs = src.mean(axis=0)
    cd = dst.mean(axis=0)
    A = src - cs
    B = dst - cd
    U, _, Vt = np.linalg.svd(A.T @ B)
    rot = Vt.T @ U.T
    if np.linalg.det(rot) < 0:
        Vt[2, :] *= -1
        rot = Vt.T @ U.T
    return make_T(rot, cd - rot @ cs)


def estimate_source_to_target(source_pose: Path, target_pose: Path) -> Tuple[np.ndarray, float, int]:
    s_ts, s_xyz = load_pose_xyz(source_pose)
    t_ts, t_xyz = load_pose_xyz(target_pose)
    mask = (s_ts >= t_ts.min()) & (s_ts <= t_ts.max())
    if np.count_nonzero(mask) >= 10:
        common_ts = s_ts[mask]
        src = s_xyz[mask]
        dst = interp_xyz(common_ts, t_ts, t_xyz)
    else:
        mask = (t_ts >= s_ts.min()) & (t_ts <= s_ts.max())
        common_ts = t_ts[mask]
        src = interp_xyz(common_ts, s_ts, s_xyz)
        dst = t_xyz[mask]
    if len(src) > 5000:
        sel = np.linspace(0, len(src) - 1, 5000).astype(int)
        src = src[sel]
        dst = dst[sel]
    T = kabsch(src, dst)
    pred = src @ T[:3, :3].T + T[:3, 3]
    rmse = float(np.sqrt(np.mean(np.sum((pred - dst) ** 2, axis=1))))
    return T, rmse, len(src)


def load_color_sources_aligned(source_roots: Iterable[Path], target_pose: Path, voxel_size: float, max_pose_rmse: float) -> Tuple[o3d.geometry.PointCloud, list]:
    merged = o3d.geometry.PointCloud()
    reports = []
    for root in source_roots:
        pose = root / "result/HKisland_GNSS02.txt"
        T, rmse, pairs = estimate_source_to_target(pose, target_pose)
        source_report = {"root": str(root), "pose_rmse_m": rmse, "pose_pairs": pairs, "used": rmse <= max_pose_rmse}
        if rmse > max_pose_rmse:
            reports.append(source_report)
            continue
        files = sorted((root / "PCD").glob("*.pcd"), key=natural_key)
        point_total = 0
        for i, path in enumerate(files, 1):
            cloud = o3d.io.read_point_cloud(str(path))
            if cloud.is_empty() or not cloud.has_colors():
                continue
            cloud.transform(T)
            merged += cloud
            point_total += len(cloud.points)
            if i % 3 == 0:
                merged = merged.voxel_down_sample(voxel_size)
        source_report["source_points_before_voxel"] = int(point_total)
        reports.append(source_report)
    if not merged.is_empty():
        merged = merged.voxel_down_sample(voxel_size)
    return merged, reports


def read_global_poses(path: Path) -> dict[str, np.ndarray]:
    poses = {}
    with path.open("r", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            bid = row["block_id"]
            trans = np.array([float(row["tx"]), float(row["ty"]), float(row["tz"])])
            rot = R.from_euler(
                "xyz",
                [float(row["roll_deg"]), float(row["pitch_deg"]), float(row["yaw_deg"])],
                degrees=True,
            ).as_matrix()
            poses[bid] = make_T(rot, trans)
    return poses


def write_ascii_rgb_ply(path: Path, pcd: o3d.geometry.PointCloud):
    xyz = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    if len(colors) != len(xyz):
        colors = np.full((len(xyz), 3), 0.5)
    rgb = np.clip(np.rint(colors * 255.0), 0, 255).astype(np.uint8)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("ply\nformat ascii 1.0\n")
        handle.write(f"element vertex {len(xyz)}\n")
        handle.write("property float x\nproperty float y\nproperty float z\n")
        handle.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        for point, color in zip(xyz, rgb):
            handle.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {int(color[0])} {int(color[1])} {int(color[2])}\n")


def build(output_path: Path, voxel_size: float, max_pose_rmse: float, final_voxel: float | None):
    source_map = {
        "block_3": [COLOR_ROOT / "block_3a_livo_color", COLOR_ROOT / "block_3b_livo_color"],
        "block_6": [COLOR_ROOT / "block_6_preheat_livo_color"],
    }
    global_poses = read_global_poses(RESULTS / "global_poses_optimized.csv")
    merged = o3d.geometry.PointCloud()
    report = {"blocks": {}}
    for bid in sorted(global_poses.keys(), key=lambda x: int(x.split("_")[-1])):
        if bid in source_map:
            local, reports = load_color_sources_aligned(source_map[bid], CORRECT_POSE / f"{bid}_pose.txt", voxel_size, max_pose_rmse)
            if local.is_empty():
                local = o3d.io.read_point_cloud(str(CORRECT_PCD / f"{bid}.pcd"))
                source = "fallback_correct_geometry"
            else:
                source = "aligned_high_density_color"
            report["blocks"][bid] = {"source": source, "local_points": int(len(local.points)), "alignment": reports}
        else:
            local = o3d.io.read_point_cloud(str(CORRECT_PCD / f"{bid}.pcd"))
            report["blocks"][bid] = {"source": "correct_colorized_geometry", "local_points": int(len(local.points))}
        local.transform(global_poses[bid])
        merged += local
        print(f"[Merge] {bid}: merged_points={len(merged.points)}")
    if final_voxel and final_voxel > 0:
        merged = merged.voxel_down_sample(final_voxel)
        report["final_voxel_m"] = final_voxel
    write_ascii_rgb_ply(output_path, merged)
    xyz = np.asarray(merged.points)
    colors = np.asarray(merged.colors)
    gray = np.zeros(len(xyz), dtype=bool)
    if len(colors) == len(xyz):
        rgb = np.clip(np.rint(colors * 255), 0, 255).astype(np.uint8)
        gray = (rgb[:, 0] == rgb[:, 1]) & (rgb[:, 1] == rgb[:, 2])
    report.update({
        "output_path": str(output_path),
        "point_count": int(len(xyz)),
        "mins": xyz.min(axis=0).astype(float).tolist(),
        "maxs": xyz.max(axis=0).astype(float).tolist(),
        "gray_fraction": float(np.mean(gray)) if len(gray) else 0.0,
        "voxel_size_m": voxel_size,
        "max_pose_rmse_m": max_pose_rmse,
    })
    output_path.with_suffix(".json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description="Build corrected high-density HKisland color map without drifting color-recovery blocks.")
    parser.add_argument("--output", required=True)
    parser.add_argument("--voxel-size", type=float, default=0.35)
    parser.add_argument("--max-pose-rmse", type=float, default=5.0)
    parser.add_argument("--final-voxel", type=float, default=0.0)
    args = parser.parse_args()
    report = build(Path(args.output), args.voxel_size, args.max_pose_rmse, args.final_voxel)
    for key, value in report.items():
        if key != "blocks":
            print(f"{key}: {value}")


if __name__ == "__main__":
    main()
