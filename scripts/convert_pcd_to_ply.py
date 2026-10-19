#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np


def read_pcd_header(handle):
    header = {}
    lines = []
    while True:
        line = handle.readline()
        if not line:
            raise ValueError("PCD header ended before DATA line")
        text = line.decode("utf-8", errors="replace").strip()
        lines.append(text)
        if not text or text.startswith("#"):
            continue
        parts = text.split()
        header[parts[0].upper()] = parts[1:]
        if parts[0].upper() == "DATA":
            break
    return header, lines


def packed_rgb_float_to_uint8(rgb_float):
    packed = np.asarray(rgb_float, dtype=np.float32).view(np.uint32)
    red = ((packed >> 16) & 255).astype(np.uint8)
    green = ((packed >> 8) & 255).astype(np.uint8)
    blue = (packed & 255).astype(np.uint8)
    return red, green, blue


def load_binary_pcd_xyzrgb(path):
    path = Path(path)
    with path.open("rb") as handle:
        header, _lines = read_pcd_header(handle)
        fields = header.get("FIELDS", [])
        sizes = [int(v) for v in header.get("SIZE", [])]
        types = header.get("TYPE", [])
        counts = [int(v) for v in header.get("COUNT", ["1"] * len(fields))]
        data_kind = header.get("DATA", [None])[0]
        if data_kind != "binary":
            raise ValueError(f"Only DATA binary is supported, got {data_kind}")
        if any(count != 1 for count in counts):
            raise ValueError("Only COUNT 1 fields are supported")
        if not {"x", "y", "z"}.issubset(fields):
            raise ValueError("PCD must contain x/y/z fields")
        point_count = int(header.get("POINTS", header.get("WIDTH"))[0])
        dtype_fields = []
        for field, size, typ in zip(fields, sizes, types):
            if typ == "F" and size == 4:
                dt = "<f4"
            elif typ == "F" and size == 8:
                dt = "<f8"
            elif typ == "U" and size == 1:
                dt = "u1"
            elif typ == "U" and size == 2:
                dt = "<u2"
            elif typ == "U" and size == 4:
                dt = "<u4"
            elif typ == "I" and size == 1:
                dt = "i1"
            elif typ == "I" and size == 2:
                dt = "<i2"
            elif typ == "I" and size == 4:
                dt = "<i4"
            else:
                raise ValueError(f"Unsupported PCD field {field}: TYPE={typ} SIZE={size}")
            dtype_fields.append((field, dt))
        cloud = np.fromfile(handle, dtype=np.dtype(dtype_fields), count=point_count)
    xyz = np.column_stack([cloud["x"], cloud["y"], cloud["z"]]).astype(np.float32)
    if "rgb" in fields:
        red, green, blue = packed_rgb_float_to_uint8(cloud["rgb"])
        rgb = np.column_stack([red, green, blue])
    else:
        rgb = np.full((len(xyz), 3), 255, dtype=np.uint8)
    return xyz, rgb


def write_ascii_ply(path, xyz, rgb):
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


def convert_pcd_to_ply(input_path, output_path):
    xyz, rgb = load_binary_pcd_xyzrgb(input_path)
    write_ascii_ply(output_path, xyz, rgb)
    return {
        "input_path": str(input_path),
        "output_path": str(output_path),
        "point_count": int(len(xyz)),
        "mins": xyz.min(axis=0).astype(float).tolist(),
        "maxs": xyz.max(axis=0).astype(float).tolist(),
        "non_gray_fraction": float(np.mean(~((rgb[:, 0] == rgb[:, 1]) & (rgb[:, 1] == rgb[:, 2])))),
    }


def main():
    parser = argparse.ArgumentParser(description="Convert binary PCD x/y/z/rgb to RGB ASCII PLY.")
    parser.add_argument("input")
    parser.add_argument("output")
    args = parser.parse_args()
    report = convert_pcd_to_ply(args.input, args.output)
    for key, value in report.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
