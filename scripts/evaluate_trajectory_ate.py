#!/usr/bin/env python3
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


EARTH_RADIUS_M = 6378137.0
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def wgs84_to_ecef(lat_deg: np.ndarray, lon_deg: np.ndarray, height_m: np.ndarray) -> np.ndarray:
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + height_m) * cos_lat * np.cos(lon)
    y = (n + height_m) * cos_lat * np.sin(lon)
    z = (n * (1.0 - WGS84_E2) + height_m) * sin_lat
    return np.column_stack([x, y, z])


def ecef_to_enu_matrix(origin_lat_deg: float, origin_lon_deg: float) -> np.ndarray:
    lat = math.radians(origin_lat_deg)
    lon = math.radians(origin_lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    return np.array([
        [-sin_lon, cos_lon, 0.0],
        [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
        [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
    ])


def geodetic_to_enu_many(lat_deg: np.ndarray, lon_deg: np.ndarray, height_m: np.ndarray, origin: tuple[float, float, float]) -> np.ndarray:
    origin_lat, origin_lon, origin_h = origin
    origin_ecef = wgs84_to_ecef(np.array([origin_lat]), np.array([origin_lon]), np.array([origin_h]))[0]
    ecef = wgs84_to_ecef(lat_deg, lon_deg, height_m)
    return (ecef - origin_ecef) @ ecef_to_enu_matrix(origin_lat, origin_lon).T


def parse_origin(text: str | None) -> tuple[float, float, float] | None:
    if not text:
        return None
    parts = [float(value) for value in text.split(',')]
    if len(parts) != 3:
        raise ValueError('--truth-origin-lat-lon-h must contain lat,lon,h')
    return (parts[0], parts[1], parts[2])


def load_origin_from_report(path: Path | None) -> tuple[float, float, float] | None:
    if path is None:
        return None
    data = json.loads(path.read_text(encoding='utf-8'))
    values = data['origin_lat_lon_h']
    return (float(values[0]), float(values[1]), float(values[2]))


def load_estimated_trajectory(path: Path) -> tuple[np.ndarray, np.ndarray]:
    rows = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if "keyframe_index" in row:
                time_value = float(row["time"])
            else:
                time_value = float(row["time"])
            rows.append([time_value, float(row["x"]), float(row["y"]), float(row["z"])])
    data = np.asarray(rows, dtype=np.float64)
    return data[:, 0], data[:, 1:4]


def load_terra_pos_as_enu(path: Path, origin: tuple[float, float, float] | None = None) -> tuple[np.ndarray, np.ndarray]:
    rows = []
    with path.open(newline="", encoding="utf-8", errors="replace") as handle:
        for row in csv.reader(handle):
            if not row or row[0].strip().startswith("#"):
                continue
            try:
                rows.append([float(value) for value in row[:4]])
            except ValueError:
                continue
    data = np.asarray(rows, dtype=np.float64)
    times = data[:, 0]
    lon_deg = data[:, 1]
    lat_deg = data[:, 2]
    alt_m = data[:, 3]
    if origin is None:
        origin = (float(lat_deg[0]), float(lon_deg[0]), float(alt_m[0]))
    return times, geodetic_to_enu_many(lat_deg, lon_deg, alt_m, origin)



def load_sbet_as_enu(path: Path, unix_offset: float, origin: tuple[float, float, float] | None = None) -> tuple[np.ndarray, np.ndarray]:
    rows = []
    with path.open(encoding="utf-8", errors="replace") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped or stripped.startswith("%"):
                continue
            values = [float(value) for value in stripped.split()]
            rows.append(values[:4])
    data = np.asarray(rows, dtype=np.float64)
    times = data[:, 0] + unix_offset
    lat_deg = np.degrees(data[:, 1])
    lon_deg = np.degrees(data[:, 2])
    alt_m = data[:, 3]
    if origin is None:
        origin = (float(lat_deg[0]), float(lon_deg[0]), float(alt_m[0]))
    return times, geodetic_to_enu_many(lat_deg, lon_deg, alt_m, origin)


def interpolate_positions(source_times: np.ndarray, source_positions: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    return np.column_stack(
        [
            np.interp(query_times, source_times, source_positions[:, axis])
            for axis in range(3)
        ]
    )


def associate_by_time(
    estimated_times: np.ndarray,
    estimated_positions: np.ndarray,
    truth_times: np.ndarray,
    truth_positions: np.ndarray,
    time_offset: float,
    max_pairs: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    shifted_times = estimated_times + time_offset
    valid = (shifted_times >= truth_times[0]) & (shifted_times <= truth_times[-1])
    valid_indices = np.flatnonzero(valid)
    if len(valid_indices) > max_pairs:
        valid_indices = valid_indices[np.linspace(0, len(valid_indices) - 1, max_pairs).astype(np.int64)]
    matched_estimated = estimated_positions[valid_indices]
    matched_truth = interpolate_positions(truth_times, truth_positions, shifted_times[valid_indices])
    return shifted_times[valid_indices], matched_estimated, matched_truth


def se3_align(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_centroid = source.mean(axis=0)
    target_centroid = target.mean(axis=0)
    source_centered = source - source_centroid
    target_centered = target - target_centroid
    covariance = source_centered.T @ target_centered
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = target_centroid - rotation @ source_centroid
    return rotation, translation


def transform(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return points @ rotation.T + translation


def compute_metrics(errors: np.ndarray) -> dict:
    return {
        "count": int(len(errors)),
        "mean": float(np.mean(errors)),
        "rmse": float(np.sqrt(np.mean(errors ** 2))),
        "median": float(np.median(errors)),
        "p68": float(np.quantile(errors, 0.68)),
        "p90": float(np.quantile(errors, 0.90)),
        "p95": float(np.quantile(errors, 0.95)),
        "p99": float(np.quantile(errors, 0.99)),
        "max": float(np.max(errors)),
    }


def evaluate_offset(
    estimated_times: np.ndarray,
    estimated_positions: np.ndarray,
    truth_times: np.ndarray,
    truth_positions: np.ndarray,
    time_offset: float,
    max_pairs: int,
) -> dict | None:
    matched_times, matched_estimated, matched_truth = associate_by_time(
        estimated_times,
        estimated_positions,
        truth_times,
        truth_positions,
        time_offset,
        max_pairs,
    )
    if len(matched_times) < 10:
        return None
    rotation, translation = se3_align(matched_estimated, matched_truth)
    aligned = transform(matched_estimated, rotation, translation)
    residuals = aligned - matched_truth
    errors = np.linalg.norm(residuals, axis=1)
    horizontal_errors = np.linalg.norm(residuals[:, :2], axis=1)
    vertical_errors = np.abs(residuals[:, 2])
    metrics = compute_metrics(errors)
    metrics["horizontal_rmse"] = float(np.sqrt(np.mean(horizontal_errors ** 2)))
    metrics["vertical_rmse"] = float(np.sqrt(np.mean(vertical_errors ** 2)))
    return {
        "time_offset_s": float(time_offset),
        "metrics": metrics,
        "rotation": rotation,
        "translation_m": translation,
        "matched_time_range": [float(matched_times[0]), float(matched_times[-1])],
    }


def search_time_offset(
    estimated_times: np.ndarray,
    estimated_positions: np.ndarray,
    truth_times: np.ndarray,
    truth_positions: np.ndarray,
    center_offset: float,
    search_radius: float,
    coarse_step: float,
    fine_step: float,
    max_pairs: int,
) -> dict:
    coarse_offsets = np.arange(center_offset - search_radius, center_offset + search_radius + coarse_step, coarse_step)
    coarse_results = [
        result
        for offset in coarse_offsets
        if (result := evaluate_offset(estimated_times, estimated_positions, truth_times, truth_positions, offset, max_pairs))
    ]
    best_coarse = min(coarse_results, key=lambda item: item["metrics"]["rmse"])
    fine_offsets = np.arange(best_coarse["time_offset_s"] - coarse_step, best_coarse["time_offset_s"] + coarse_step + fine_step, fine_step)
    fine_results = [
        result
        for offset in fine_offsets
        if (result := evaluate_offset(estimated_times, estimated_positions, truth_times, truth_positions, offset, max_pairs))
    ]
    return min(fine_results, key=lambda item: item["metrics"]["rmse"])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--estimate", required=True, type=Path)
    parser.add_argument("--truth", required=True, type=Path)
    parser.add_argument("--truth-format", choices=["pos", "sbet"], default="pos")
    parser.add_argument("--sbet-unix-offset", type=float, default=1697932800.0)
    parser.add_argument("--truth-origin-lat-lon-h", type=str, default=None)
    parser.add_argument("--truth-origin-report", type=Path, default=None)
    parser.add_argument("--out-json", required=True, type=Path)
    parser.add_argument("--out-md", required=True, type=Path)
    parser.add_argument("--max-pairs", type=int, default=20000)
    parser.add_argument("--search-radius", type=float, default=120.0)
    parser.add_argument("--coarse-step", type=float, default=1.0)
    parser.add_argument("--fine-step", type=float, default=0.02)
    args = parser.parse_args()

    estimated_times, estimated_positions = load_estimated_trajectory(args.estimate)
    truth_origin = parse_origin(args.truth_origin_lat_lon_h) or load_origin_from_report(args.truth_origin_report)
    origin_text = "provided estimate GNSS origin" if truth_origin is not None else "first truth pose"
    if args.truth_format == "sbet":
        truth_times, truth_positions = load_sbet_as_enu(args.truth, args.sbet_unix_offset, truth_origin)
        truth_conversion = f"DJI Terra SBET SOW + GPS-week Unix offset converted to local ENU using {origin_text}"
    else:
        truth_times, truth_positions = load_terra_pos_as_enu(args.truth, truth_origin)
        truth_conversion = f"DJI Terra POS lon/lat/alt converted to local ENU using {origin_text}"
    center_offset = truth_times[0] - estimated_times[0]
    result = search_time_offset(
        estimated_times,
        estimated_positions,
        truth_times,
        truth_positions,
        center_offset,
        args.search_radius,
        args.coarse_step,
        args.fine_step,
        args.max_pairs,
    )
    report = {
        "estimate_file": str(args.estimate),
        "truth_file": str(args.truth),
        "truth_conversion": truth_conversion,
        "truth_origin_lat_lon_h": list(truth_origin) if truth_origin is not None else None,
        "alignment": "SE(3) rigid alignment after timestamp association; no scale correction",
        "time_offset_search": {
            "center_offset_s": float(center_offset),
            "search_radius_s": args.search_radius,
            "coarse_step_s": args.coarse_step,
            "fine_step_s": args.fine_step,
        },
        "best_time_offset_s": result["time_offset_s"],
        "matched_time_range": result["matched_time_range"],
        "rotation": result["rotation"].tolist(),
        "translation_m": result["translation_m"].tolist(),
        "ate_m": result["metrics"],
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    metrics = report["ate_m"]
    lines = [
        "# 香港岛轨迹 ATE 评估（DJI Terra POS 作为真值）",
        "",
        "## 数据",
        f"- 估计轨迹：`{report['estimate_file']}`",
        f"- 真值轨迹：`{report['truth_file']}`",
        "- 真值转换：经纬高转局部 ENU，第一帧真值为原点",
        "- 对齐方式：时间偏移搜索 + SE(3) 刚体对齐，不做尺度缩放",
        "",
        "## ATE 结果（m）",
    ]
    for key in ["rmse", "mean", "median", "p68", "p90", "p95", "p99", "max", "horizontal_rmse", "vertical_rmse"]:
        lines.append(f"- {key}: {metrics[key]:.4f}")
    lines.extend(
        [
            "",
            "## 时间配准",
            f"- 最优时间偏移：{report['best_time_offset_s']:.4f} s",
            f"- 匹配点数：{metrics['count']}",
            f"- 匹配真值时间范围：{report['matched_time_range'][0]:.3f} - {report['matched_time_range'][1]:.3f}",
        ]
    )
    args.out_md.write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps({"best_time_offset_s": report["best_time_offset_s"], "ate_m": metrics}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
