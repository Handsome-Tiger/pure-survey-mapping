from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from survey_mapping.geo import quat_to_matrix
from survey_mapping.optimized_mapping import PoseInterpolator


@dataclass
class RegistrationResult:
    transform: np.ndarray
    rmse: float
    inlier_fraction: float
    converged: bool
    iterations: int


def deskew_points(points, frame_time, pose_series, reference_time, rotation_body_lidar, translation_body_lidar, navigation_time_offset_s=0.0):
    points = np.asarray(points, dtype=float)
    if len(points) == 0:
        return np.zeros((0, 4), dtype=float)
    r_bl = np.asarray(rotation_body_lidar, dtype=float)
    t_bl = np.asarray(translation_body_lidar, dtype=float)
    interpolator = PoseInterpolator(
        pose_series.times,
        np.asarray([pose.position for pose in pose_series.poses]),
        np.asarray([pose.quaternion for pose in pose_series.poses]),
    )
    navigation_time_offset_s = float(navigation_time_offset_s)
    point_times = float(frame_time) + points[:, 0] * 1e-9 + navigation_time_offset_s
    points_body = (r_bl @ points[:, 1:4].T).T + t_bl
    points_world = interpolator.transform_body_points(points_body, point_times)
    reference_position, reference_quaternion = interpolator.sample_many(
        np.array([float(reference_time) + navigation_time_offset_s])
    )
    points_body_reference = Rotation.from_quat(reference_quaternion[0]).inv().apply(
        points_world - reference_position[0]
    )
    points_lidar_reference = (r_bl.T @ (points_body_reference - t_bl).T).T
    return np.column_stack([points_lidar_reference, points[:, 4]])


def voxel_centroids(points_xyz, voxel_size):
    points = np.asarray(points_xyz, dtype=float)
    if len(points) == 0:
        return np.zeros((0, 3), dtype=float)
    keys = np.floor(points / float(voxel_size)).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    sums = np.zeros((int(inverse.max()) + 1, 3), dtype=float)
    counts = np.zeros(len(sums), dtype=np.int64)
    np.add.at(sums, inverse, points)
    np.add.at(counts, inverse, 1)
    return sums / counts[:, None]


def estimate_normals(points_xyz, neighbors=16):
    points = np.asarray(points_xyz, dtype=float)
    if len(points) < 3:
        return np.zeros_like(points)
    count = min(max(3, int(neighbors)), len(points))
    tree = cKDTree(points)
    _, indices = tree.query(points, k=count)
    normals = np.zeros_like(points)
    for index, neighborhood in enumerate(indices):
        local = points[np.atleast_1d(neighborhood)]
        centered = local - local.mean(axis=0)
        covariance = centered.T @ centered / max(1, len(local) - 1)
        _values, vectors = np.linalg.eigh(covariance)
        normals[index] = vectors[:, 0]
    return normals


def register_point_to_plane(
    source_xyz,
    target_xyz,
    initial_transform=None,
    max_distance=1.5,
    iterations=30,
    normal_neighbors=16,
    huber_delta=0.2,
    tolerance=1e-5,
):
    source = np.asarray(source_xyz, dtype=float)
    target = np.asarray(target_xyz, dtype=float)
    transform = np.eye(4, dtype=float) if initial_transform is None else np.asarray(initial_transform, dtype=float).copy()
    if len(source) < 6 or len(target) < 6:
        return RegistrationResult(transform, float("inf"), 0.0, False, 0)

    tree = cKDTree(target)
    normals = estimate_normals(target, normal_neighbors)
    converged = False
    used_iterations = 0
    rmse = float("inf")
    inlier_fraction = 0.0

    for iteration in range(int(iterations)):
        transformed = (transform[:3, :3] @ source.T).T + transform[:3, 3]
        distances, indices = tree.query(transformed, k=1)
        keep = np.isfinite(distances) & (distances <= float(max_distance))
        inlier_fraction = float(np.count_nonzero(keep) / len(source))
        if np.count_nonzero(keep) < 6:
            break

        points = transformed[keep]
        matches = target[indices[keep]]
        matched_normals = normals[indices[keep]]
        residuals = np.einsum("ij,ij->i", matched_normals, points - matches)
        jacobian = np.column_stack([np.cross(points, matched_normals), matched_normals])
        absolute = np.abs(residuals)
        weights = np.ones_like(absolute)
        large = absolute > float(huber_delta)
        weights[large] = float(huber_delta) / absolute[large]
        root_weights = np.sqrt(weights)
        delta, *_ = np.linalg.lstsq(jacobian * root_weights[:, None], -residuals * root_weights, rcond=None)

        delta_rotation = Rotation.from_rotvec(delta[:3]).as_matrix()
        delta_translation = delta[3:6]
        transform[:3, :3] = delta_rotation @ transform[:3, :3]
        transform[:3, 3] = delta_rotation @ transform[:3, 3] + delta_translation
        rmse = float(np.sqrt(np.mean(residuals * residuals)))
        used_iterations = iteration + 1
        if np.linalg.norm(delta) < float(tolerance):
            converged = True
            break

    return RegistrationResult(transform, rmse, inlier_fraction, converged, used_iterations)
