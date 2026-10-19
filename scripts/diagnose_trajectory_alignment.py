#!/usr/bin/env python3
import csv
import glob
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


A = 6378137.0
F = 1 / 298.257223563
E2 = F * (2 - F)


def ecef(latd, lond, h):
    lat = np.radians(latd)
    lon = np.radians(lond)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    n = A / np.sqrt(1 - E2 * sin_lat * sin_lat)
    return np.c_[
        (n + h) * cos_lat * np.cos(lon),
        (n + h) * cos_lat * np.sin(lon),
        (n * (1 - E2) + h) * sin_lat,
    ]


def enu_matrix(latd, lond):
    lat = math.radians(latd)
    lon = math.radians(lond)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    return np.array(
        [
            [-sin_lon, cos_lon, 0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ]
    )


def to_enu(lat, lon, h, origin):
    origin_ecef = ecef(np.array([origin[0]]), np.array([origin[1]]), np.array([origin[2]]))[0]
    return (ecef(lat, lon, h) - origin_ecef) @ enu_matrix(origin[0], origin[1]).T


def load_xyz_csv(path):
    return np.array(
        [
            [float(row["time"]), float(row["x"]), float(row["y"]), float(row["z"])]
            for row in csv.DictReader(open(path, newline=""))
        ]
    )


def load_keyframes(path):
    return np.array(
        [
            [float(row["time"]), float(row["x"]), float(row["y"]), float(row["z"])]
            for row in csv.DictReader(open(path, newline=""))
        ]
    )


def load_sbet(origin):
    paths = glob.glob(
        "/mnt/e/*/DJI/DJITerra/19834212827/*/lidars/terra_trajectory/merge/*20231024175427*sbet.txt"
    )
    path = Path(paths[-1])
    rows = []
    for line in path.open(errors="replace"):
        stripped = line.strip()
        if stripped and not stripped.startswith("%"):
            rows.append([float(value) for value in stripped.split()[:4]])
    data = np.array(rows)
    positions = to_enu(np.degrees(data[:, 1]), np.degrees(data[:, 2]), data[:, 3], origin)
    return data[:, 0] + 1697932800.0, positions


def metrics(errors):
    return {
        "rmse": float(np.sqrt(np.mean(errors * errors))),
        "median": float(np.median(errors)),
        "p95": float(np.quantile(errors, 0.95)),
        "max": float(np.max(errors)),
    }


def arclen(points):
    return np.r_[0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]


def resample(points, count):
    distances = arclen(points)
    query = np.linspace(0, distances[-1], count)
    return np.c_[
        np.interp(query, distances, points[:, 0]),
        np.interp(query, distances, points[:, 1]),
        np.interp(query, distances, points[:, 2]),
    ]


def align(source, target):
    source_centroid = source.mean(0)
    target_centroid = target.mean(0)
    source_centered = source - source_centroid
    target_centered = target - target_centroid
    u, _, vt = np.linalg.svd(source_centered.T @ target_centered)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    translation = target_centroid - rotation @ source_centroid
    return source @ rotation.T + translation


def main():
    root = Path("/home/shuai/cehui")
    out = root / "outputs_joint_kfgins_all_livox_deskew_color"
    origin = tuple(json.load(open(out / "keyframe_report.json"))["origin_lat_lon_h"])
    keyframes = load_keyframes(out / "trajectory_optimized_keyframes.csv")[:, 1:4]
    gnss = load_xyz_csv(out / "trajectory_gnss.csv")[:, 1:4]
    nav = load_xyz_csv(out / "trajectory_nav_initial.csv")[::400, 1:4]
    _, sbet = load_sbet(origin)

    print("origin", origin)
    print("span key/gnss/sbet", np.ptp(keyframes, axis=0), np.ptp(gnss, axis=0), np.ptp(sbet, axis=0))
    tree = cKDTree(sbet)
    for name, points in [("keyframes", keyframes), ("gnss", gnss), ("nav_sub", nav)]:
        distances, indices = tree.query(points)
        residuals = points - sbet[indices]
        horizontal = np.linalg.norm(residuals[:, :2], axis=1)
        vertical = np.abs(residuals[:, 2])
        print(name, "3d", metrics(distances), "horizontal", metrics(horizontal), "vertical", metrics(vertical))

    for reverse in [False, True]:
        truth = resample(sbet[::-1] if reverse else sbet, len(keyframes))
        aligned = align(keyframes, truth)
        errors = np.linalg.norm(aligned - truth, axis=1)
        print("arclength keyframes reverse", reverse, metrics(errors))


if __name__ == "__main__":
    main()
