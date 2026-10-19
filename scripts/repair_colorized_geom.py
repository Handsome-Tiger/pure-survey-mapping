#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def read_rgb_ply(path):
    path = Path(path)
    header = []
    count = None
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            header.append(line.rstrip())
            if line.startswith("element vertex"):
                count = int(line.split()[2])
            if line.strip() == "end_header":
                break
        xyz = np.empty((count, 3), dtype=np.float32)
        rgb = np.empty((count, 3), dtype=np.uint8)
        for idx, line in enumerate(handle):
            parts = line.split()
            if len(parts) < 6:
                continue
            xyz[idx] = [float(parts[0]), float(parts[1]), float(parts[2])]
            rgb[idx] = [int(parts[3]), int(parts[4]), int(parts[5])]
    return xyz, rgb


def write_rgb_ply(path, xyz, rgb):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("ply\n")
        handle.write("format ascii 1.0\n")
        handle.write(f"element vertex {len(xyz)}\n")
        handle.write("property float x\n")
        handle.write("property float y\n")
        handle.write("property float z\n")
        handle.write("property uchar red\n")
        handle.write("property uchar green\n")
        handle.write("property uchar blue\n")
        handle.write("end_header\n")
        for point, color in zip(xyz, rgb):
            handle.write(
                f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
            )


def repair(input_path, recolored_path, nogray_path, gray_min=40, gray_max=220, radius=3.0):
    xyz, rgb = read_rgb_ply(input_path)
    exact_gray = (rgb[:, 0] == rgb[:, 1]) & (rgb[:, 1] == rgb[:, 2])
    fill_mask = exact_gray & (rgb[:, 0] >= gray_min) & (rgb[:, 0] <= gray_max)
    source_mask = ~exact_gray
    repaired = rgb.copy()
    filled = 0
    unresolved = int(fill_mask.sum())
    if np.any(fill_mask) and np.any(source_mask):
        tree = cKDTree(xyz[source_mask])
        dist, idx = tree.query(xyz[fill_mask], k=1, distance_upper_bound=radius)
        valid = np.isfinite(dist)
        source_rgb = rgb[source_mask]
        fill_indices = np.flatnonzero(fill_mask)
        repaired[fill_indices[valid]] = source_rgb[idx[valid]]
        filled = int(np.count_nonzero(valid))
        unresolved = int(np.count_nonzero(~valid))
    keep = ~fill_mask
    write_rgb_ply(recolored_path, xyz, repaired)
    write_rgb_ply(nogray_path, xyz[keep], rgb[keep])
    return {
        "input_points": int(len(xyz)),
        "exact_gray_points": int(exact_gray.sum()),
        "target_gray_points": int(fill_mask.sum()),
        "filled_gray_points": filled,
        "unresolved_gray_points": unresolved,
        "recolored_path": str(recolored_path),
        "nogray_path": str(nogray_path),
        "nogray_points": int(np.count_nonzero(keep)),
    }


def main():
    parser = argparse.ArgumentParser(description="Repair gray fallback colors in an RGB PLY by nearest colored neighbors.")
    parser.add_argument("input")
    parser.add_argument("--recolored", required=True)
    parser.add_argument("--nogray", required=True)
    parser.add_argument("--radius", type=float, default=3.0)
    args = parser.parse_args()
    report = repair(args.input, args.recolored, args.nogray, radius=args.radius)
    for key, value in report.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
