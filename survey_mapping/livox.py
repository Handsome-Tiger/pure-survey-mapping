from pathlib import Path

import numpy as np

from survey_mapping.geo import quat_to_matrix


def filter_valid_points(points, min_range=0.1):
    points = np.asarray(points, dtype=float)
    if points.size == 0:
        return np.zeros((0, 7), dtype=float)
    return points[np.linalg.norm(points[:, 1:4], axis=1) > float(min_range)]


def georeference_points(points, frame_start_time, pose_series, extrinsic_rotation=None, extrinsic_translation=None):
    points = np.asarray(points, dtype=float)
    if points.size == 0:
        return np.zeros((0, 4), dtype=float)
    r_bl = np.eye(3) if extrinsic_rotation is None else np.asarray(extrinsic_rotation, dtype=float)
    t_bl = np.zeros(3) if extrinsic_translation is None else np.asarray(extrinsic_translation, dtype=float)
    mapped = np.zeros((len(points), 4), dtype=float)
    for idx, point in enumerate(points):
        point_time = float(frame_start_time) + float(point[0]) * 1e-9
        pose = pose_series.sample(point_time)
        r_mb = quat_to_matrix(pose.quaternion)
        local = r_bl @ point[1:4] + t_bl
        mapped[idx, :3] = r_mb @ local + pose.position
        mapped[idx, 3] = point[4]
    return mapped


def write_ply(path, points):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    points = np.asarray(points, dtype=float)
    with path.open("w", encoding="utf-8") as handle:
        handle.write("ply\n")
        handle.write("format ascii 1.0\n")
        handle.write(f"element vertex {len(points)}\n")
        handle.write("property float x\n")
        handle.write("property float y\n")
        handle.write("property float z\n")
        handle.write("property uchar intensity\n")
        handle.write("end_header\n")
        for x, y, z, intensity in points:
            clipped = int(max(0, min(255, round(float(intensity)))))
            handle.write(f"{x:.6f} {y:.6f} {z:.6f} {clipped}\n")
