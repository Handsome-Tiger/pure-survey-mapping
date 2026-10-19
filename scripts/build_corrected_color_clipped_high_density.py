#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

REPO = Path("/home/shuai/code/mapping-fusion-main")
FAST_WS = Path("/home/shuai/code/fast-livo2-humble-ws")
CORRECT_PCD = REPO / "data/HK_PCD_HKISLAND_COLORIZED_GEOM"
RESULTS = REPO / "results_hkisland_blocks_colorized_geom"
COLOR_ROOT = FAST_WS / "runtime/hkisland_blocks_livo_color_recovery"


def natural_key(path: Path):
    try:
        return int(path.stem)
    except ValueError:
        return path.stem


def make_T(rot, trans):
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = trans
    return T


def read_global_poses(path: Path) -> dict[str, np.ndarray]:
    poses = {}
    with path.open("r", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rot = R.from_euler(
                "xyz",
                [float(row["roll_deg"]), float(row["pitch_deg"]), float(row["yaw_deg"])],
                degrees=True,
            ).as_matrix()
            trans = np.array([float(row["tx"]), float(row["ty"]), float(row["tz"])])
            poses[row["block_id"]] = make_T(rot, trans)
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


def load_clipped_sources(block_id: str, source_dirs: list[Path], support: o3d.geometry.PointCloud, threshold: float, batch_voxel: float) -> tuple[o3d.geometry.PointCloud, list]:
    support_pts = np.asarray(support.points)
    tree = cKDTree(support_pts)
    clipped = o3d.geometry.PointCloud()
    reports = []
    for source_dir in source_dirs:
        files = sorted((source_dir / "PCD").glob("*.pcd"), key=natural_key)
        source_in = 0
        source_kept = 0
        for i, file_path in enumerate(files, 1):
            cloud = o3d.io.read_point_cloud(str(file_path))
            if cloud.is_empty() or not cloud.has_colors():
                continue
            pts = np.asarray(cloud.points)
            colors = np.asarray(cloud.colors)
            dists, _ = tree.query(pts, k=1, workers=-1)
            keep = dists <= threshold
            source_in += len(pts)
            source_kept += int(np.count_nonzero(keep))
            if np.any(keep):
                part = o3d.geometry.PointCloud()
                part.points = o3d.utility.Vector3dVector(pts[keep])
                part.colors = o3d.utility.Vector3dVector(colors[keep])
                clipped += part
            if batch_voxel > 0 and i % 3 == 0 and len(clipped.points) > 0:
                clipped = clipped.voxel_down_sample(batch_voxel)
        reports.append({
            "source": str(source_dir),
            "input_points": int(source_in),
            "kept_points_before_voxel": int(source_kept),
            "threshold_m": float(threshold),
        })
    if batch_voxel > 0 and len(clipped.points) > 0:
        clipped = clipped.voxel_down_sample(batch_voxel)
    return clipped, reports


def color_repair_gray(pcd: o3d.geometry.PointCloud, radius: float = 3.0):
    pts = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    if len(pts) == 0 or len(colors) != len(pts):
        return {"filled": 0, "target_gray": 0}
    rgb = np.clip(np.rint(colors * 255.0), 0, 255).astype(np.uint8)
    gray = (rgb[:, 0] == rgb[:, 1]) & (rgb[:, 1] == rgb[:, 2]) & (rgb[:, 0] >= 40) & (rgb[:, 0] <= 220)
    source = ~((rgb[:, 0] == rgb[:, 1]) & (rgb[:, 1] == rgb[:, 2]))
    if not np.any(gray) or not np.any(source):
        return {"filled": 0, "target_gray": int(np.count_nonzero(gray))}
    tree = cKDTree(pts[source])
    d, idx = tree.query(pts[gray], k=1, distance_upper_bound=radius, workers=-1)
    valid = np.isfinite(d)
    source_colors = colors[source]
    gray_idx = np.flatnonzero(gray)
    colors[gray_idx[valid]] = source_colors[idx[valid]]
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return {"filled": int(np.count_nonzero(valid)), "target_gray": int(np.count_nonzero(gray))}


def build(output_path: Path, threshold: float, batch_voxel: float, local_final_voxel: float, repair_colors: bool):
    high_sources = {
        "block_3": [COLOR_ROOT / "block_3a_livo_color", COLOR_ROOT / "block_3b_livo_color"],
        "block_6": [COLOR_ROOT / "block_6_preheat_livo_color"],
    }
    poses = read_global_poses(RESULTS / "global_poses_optimized.csv")
    merged = o3d.geometry.PointCloud()
    report = {"blocks": {}}
    for bid in sorted(poses.keys(), key=lambda x: int(x.split("_")[-1])):
        support = o3d.io.read_point_cloud(str(CORRECT_PCD / f"{bid}.pcd"))
        local = o3d.geometry.PointCloud(support)
        block_report = {"support_points": int(len(support.points))}
        if bid in high_sources:
            clipped, source_reports = load_clipped_sources(bid, high_sources[bid], support, threshold, batch_voxel)
            block_report["clipped_source_points"] = int(len(clipped.points))
            block_report["sources"] = source_reports
            if len(clipped.points) > 0:
                local += clipped
        if local_final_voxel > 0:
            local = local.voxel_down_sample(local_final_voxel)
        if repair_colors:
            block_report["color_repair"] = color_repair_gray(local)
        block_report["local_points"] = int(len(local.points))
        local.transform(poses[bid])
        merged += local
        report["blocks"][bid] = block_report
        print(f"[Merge] {bid}: local={block_report['local_points']} merged={len(merged.points)}")
    write_ascii_rgb_ply(output_path, merged)
    xyz = np.asarray(merged.points)
    colors = np.asarray(merged.colors)
    rgb = np.clip(np.rint(colors * 255.0), 0, 255).astype(np.uint8) if len(colors) == len(xyz) else np.zeros((0, 3), dtype=np.uint8)
    gray = (rgb[:, 0] == rgb[:, 1]) & (rgb[:, 1] == rgb[:, 2]) if len(rgb) else np.array([])
    report.update({
        "output_path": str(output_path),
        "point_count": int(len(xyz)),
        "threshold_m": float(threshold),
        "batch_voxel_m": float(batch_voxel),
        "local_final_voxel_m": float(local_final_voxel),
        "gray_fraction": float(np.mean(gray)) if len(gray) else 0.0,
        "mins": xyz.min(axis=0).astype(float).tolist(),
        "maxs": xyz.max(axis=0).astype(float).tolist(),
    })
    output_path.with_suffix(".json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description="Correct Global_Map_Optimized_Color by clipping high-density color sources to correct geometry support.")
    parser.add_argument("--output", required=True)
    parser.add_argument("--threshold", type=float, default=2.0)
    parser.add_argument("--batch-voxel", type=float, default=0.25)
    parser.add_argument("--local-final-voxel", type=float, default=0.0)
    parser.add_argument("--no-repair-colors", action="store_true")
    args = parser.parse_args()
    report = build(Path(args.output), args.threshold, args.batch_voxel, args.local_final_voxel, not args.no_repair_colors)
    for k, v in report.items():
        if k != "blocks":
            print(f"{k}: {v}")


if __name__ == "__main__":
    main()
