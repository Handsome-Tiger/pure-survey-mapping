#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def read_rgb_ply(path):
    path = Path(path)
    count = None
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.startswith("element vertex"):
                count = int(line.split()[2])
            if line.strip() == "end_header":
                break
        xyz = np.empty((count, 3), dtype=np.float32)
        rgb = np.empty((count, 3), dtype=np.uint8)
        idx = 0
        for line in handle:
            parts = line.split()
            if len(parts) < 6:
                continue
            xyz[idx] = [float(parts[0]), float(parts[1]), float(parts[2])]
            rgb[idx] = [int(parts[3]), int(parts[4]), int(parts[5])]
            idx += 1
    if idx != count:
        xyz = xyz[:idx]
        rgb = rgb[:idx]
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


def densify(input_path, output_path, radius=1.5, neighbors=4, max_new=4000000):
    xyz, rgb = read_rgb_ply(input_path)
    tree = cKDTree(xyz)
    distances, indices = tree.query(xyz, k=neighbors + 1, distance_upper_bound=radius)
    pair_chunks = []
    n = len(xyz)
    base = np.arange(n, dtype=np.int64)
    for rank in range(1, neighbors + 1):
        j = indices[:, rank].astype(np.int64)
        d = distances[:, rank]
        valid = (j < n) & np.isfinite(d) & (d > 1e-6) & (base < j)
        if np.any(valid):
            pair_chunks.append(np.column_stack([base[valid], j[valid]]))
    if pair_chunks:
        pairs = np.vstack(pair_chunks)
        pairs = np.unique(pairs, axis=0)
        if max_new and len(pairs) > max_new:
            pairs = pairs[:max_new]
        new_xyz = (xyz[pairs[:, 0]] + xyz[pairs[:, 1]]) * 0.5
        new_rgb = ((rgb[pairs[:, 0]].astype(np.uint16) + rgb[pairs[:, 1]].astype(np.uint16)) // 2).astype(np.uint8)
        out_xyz = np.vstack([xyz, new_xyz])
        out_rgb = np.vstack([rgb, new_rgb])
    else:
        pairs = np.zeros((0, 2), dtype=np.int64)
        out_xyz = xyz
        out_rgb = rgb
    write_rgb_ply(output_path, out_xyz, out_rgb)
    return {
        "input_points": int(len(xyz)),
        "new_points": int(len(pairs)),
        "output_points": int(len(out_xyz)),
        "radius_m": float(radius),
        "neighbors": int(neighbors),
        "output_path": str(output_path),
    }


def main():
    parser = argparse.ArgumentParser(description="Densify an RGB PLY by inserting local nearest-neighbor midpoints.")
    parser.add_argument("input")
    parser.add_argument("output")
    parser.add_argument("--radius", type=float, default=1.5)
    parser.add_argument("--neighbors", type=int, default=4)
    parser.add_argument("--max-new", type=int, default=4000000)
    args = parser.parse_args()
    report = densify(args.input, args.output, args.radius, args.neighbors, args.max_new)
    for key, value in report.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
