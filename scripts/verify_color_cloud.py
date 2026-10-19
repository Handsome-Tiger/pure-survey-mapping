#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from scripts.build_optimized_color_cloud import color_output_paths
from survey_mapping.io import write_json


COMPLETE_DTYPE = np.dtype(
    [("xyz", "<f4", (3,)), ("rgb", "u1", (3,)), ("color_valid", "u1")]
)
STRICT_DTYPE = np.dtype([("xyz", "<f4", (3,)), ("rgb", "u1", (3,))])
BASE_PROPERTIES = [
    ("float", "x"),
    ("float", "y"),
    ("float", "z"),
    ("uchar", "red"),
    ("uchar", "green"),
    ("uchar", "blue"),
]


def _parse_header(handle):
    if handle.readline() != b"ply\n":
        raise ValueError("not a PLY file")
    if handle.readline() != b"format binary_little_endian 1.0\n":
        raise ValueError("PLY must use binary_little_endian 1.0")
    declared_count = None
    properties = []
    while True:
        line = handle.readline()
        if not line:
            raise ValueError("PLY header is missing end_header")
        text = line.decode("ascii").strip()
        if text == "end_header":
            break
        fields = text.split()
        if fields[:2] == ["element", "vertex"] and len(fields) == 3:
            declared_count = int(fields[2])
        elif fields[:1] == ["property"] and len(fields) == 3:
            properties.append((fields[1], fields[2]))
    if declared_count is None:
        raise ValueError("PLY header does not declare vertices")
    return declared_count, properties, handle.tell()


def _verify_ply(path, dtype, expected_properties, complete):
    path = Path(path)
    with path.open("rb") as handle:
        declared_count, properties, payload_offset = _parse_header(handle)
        if properties != expected_properties:
            raise ValueError(f"unexpected PLY vertex properties: {properties}")
        payload_bytes = path.stat().st_size - payload_offset
        if payload_bytes < 0 or payload_bytes % dtype.itemsize != 0:
            raise ValueError("PLY declared vertex count does not match binary payload")
        actual_count = payload_bytes // dtype.itemsize
        if actual_count != declared_count:
            raise ValueError("PLY declared vertex count does not match binary payload")
        records = np.fromfile(handle, dtype=dtype, count=actual_count)

    finite = np.all(np.isfinite(records["xyz"]), axis=1)
    finite_count = int(np.count_nonzero(finite))
    if finite_count != actual_count:
        raise ValueError("PLY contains non-finite XYZ coordinates")
    if complete:
        validity = records["color_valid"]
        if not np.all((validity == 0) | (validity == 1)):
            raise ValueError("PLY color_valid values must be 0 or 1")
        true_color_count = int(np.count_nonzero(validity))
    else:
        true_color_count = int(actual_count)

    minimum = records["xyz"].min(axis=0).astype(float).tolist() if actual_count else None
    maximum = records["xyz"].max(axis=0).astype(float).tolist() if actual_count else None
    return {
        "path": str(path),
        "declared_vertex_count": int(declared_count),
        "actual_vertex_count": int(actual_count),
        "finite_vertex_count": finite_count,
        "true_color_count": true_color_count,
        "minimum_xyz": minimum,
        "maximum_xyz": maximum,
        "file_size_bytes": int(path.stat().st_size),
        "record_size_bytes": int(dtype.itemsize),
    }


def verify_complete_ply(path):
    return _verify_ply(
        path,
        COMPLETE_DTYPE,
        BASE_PROPERTIES + [("uchar", "color_valid")],
        complete=True,
    )


def verify_strict_ply(path):
    return _verify_ply(path, STRICT_DTYPE, BASE_PROPERTIES, complete=False)


def verify_configured_clouds(config):
    complete_path, strict_path = color_output_paths(config)
    complete = verify_complete_ply(complete_path)
    strict = verify_strict_ply(strict_path)
    if strict["actual_vertex_count"] != complete["true_color_count"]:
        raise ValueError("strict PLY count does not match complete PLY true-color count")
    result = {
        "complete": complete,
        "strict": strict,
        "counts_consistent": True,
    }
    output_root = Path(config["outputs"]["root"])
    write_json(output_root / "color_cloud_verification.json", result)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    result = verify_configured_clouds(config)
    for name, values in result.items():
        print(f"{name}: {values}")


if __name__ == "__main__":
    main()

