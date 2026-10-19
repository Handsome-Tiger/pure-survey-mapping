import numpy as np
from scipy.spatial.transform import Rotation


def _quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.array(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ],
        dtype=float,
    )


def _interpolate_quaternions(times, quaternions, sample_times):
    sample_times = np.asarray(sample_times, dtype=float)
    indices = np.searchsorted(times, sample_times, side="right")
    indices = np.clip(indices, 1, len(times) - 1)
    first = quaternions[indices - 1].copy()
    second = quaternions[indices].copy()
    flip = np.einsum("ij,ij->i", first, second) < 0.0
    second[flip] *= -1.0
    duration = times[indices] - times[indices - 1]
    ratio = np.divide(
        sample_times - times[indices - 1],
        duration,
        out=np.zeros_like(sample_times),
        where=duration > 0.0,
    )
    ratio = np.clip(ratio, 0.0, 1.0)[:, None]
    result = first * (1.0 - ratio) + second * ratio
    result /= np.linalg.norm(result, axis=1, keepdims=True)
    result[sample_times <= times[0]] = quaternions[0]
    result[sample_times >= times[-1]] = quaternions[-1]
    return result


class ImuRotationSeries:
    def __init__(self, times, quaternions):
        self.times = np.asarray(times, dtype=float)
        self.quaternions = np.asarray(quaternions, dtype=float)
        if self.times.ndim != 1 or self.quaternions.shape != (len(self.times), 4):
            raise ValueError("IMU times and quaternions must have shapes (N,) and (N, 4)")
        if len(self.times) < 2 or not np.all(np.diff(self.times) > 0.0):
            raise ValueError("IMU timestamps must be strictly increasing")

    @classmethod
    def from_gyro(cls, times, gyro, bias=None):
        times = np.asarray(times, dtype=float)
        gyro = np.asarray(gyro, dtype=float)
        if times.ndim != 1 or gyro.shape != (len(times), 3):
            raise ValueError("IMU times and gyro must have shapes (N,) and (N, 3)")
        if len(times) < 2 or not np.all(np.diff(times) > 0.0):
            raise ValueError("IMU timestamps must be strictly increasing")
        if not np.all(np.isfinite(times)) or not np.all(np.isfinite(gyro)):
            raise ValueError("IMU samples must be finite")
        bias = np.zeros(3, dtype=float) if bias is None else np.asarray(bias, dtype=float)
        if bias.shape != (3,):
            raise ValueError("gyroscope bias must have shape (3,)")

        quaternions = np.zeros((len(times), 4), dtype=float)
        quaternions[0, 3] = 1.0
        midpoint_rates = 0.5 * (gyro[:-1] + gyro[1:]) - bias
        delta_rotations = Rotation.from_rotvec(midpoint_rates * np.diff(times)[:, None]).as_quat()
        for index, delta in enumerate(delta_rotations, start=1):
            quaternions[index] = _quaternion_multiply(quaternions[index - 1], delta)
            quaternions[index] /= np.linalg.norm(quaternions[index])
        return cls(times, quaternions)

    def sample_many(self, sample_times):
        return _interpolate_quaternions(self.times, self.quaternions, sample_times)

    def relative_rotations(self, sample_times, reference_time):
        sample_quaternions = self.sample_many(sample_times)
        reference_quaternion = self.sample_many(np.array([float(reference_time)]))[0]
        reference_rotation = Rotation.from_quat(reference_quaternion)
        return reference_rotation.inv() * Rotation.from_quat(sample_quaternions)


def deskew_points_with_imu(
    points,
    frame_time,
    reference_time,
    imu_rotations,
    pose_series,
    navigation_time_offset_s=0.0,
    rotation_body_lidar=None,
    translation_body_lidar=None,
    rotation_imu_lidar=None,
    translation_imu_lidar=None,
):
    """Express Livox returns in the LiDAR frame at ``reference_time``."""
    points = np.asarray(points, dtype=float)
    if points.size == 0:
        return np.zeros((0, 4), dtype=float)
    rotation_body_lidar = (
        np.eye(3) if rotation_body_lidar is None else np.asarray(rotation_body_lidar, dtype=float)
    )
    translation_body_lidar = (
        np.zeros(3)
        if translation_body_lidar is None
        else np.asarray(translation_body_lidar, dtype=float)
    )
    rotation_imu_lidar = (
        np.eye(3) if rotation_imu_lidar is None else np.asarray(rotation_imu_lidar, dtype=float)
    )
    translation_imu_lidar = (
        np.zeros(3)
        if translation_imu_lidar is None
        else np.asarray(translation_imu_lidar, dtype=float)
    )

    from survey_mapping.optimized_mapping import PoseInterpolator

    navigation = PoseInterpolator(
        pose_series.times,
        np.asarray([pose.position for pose in pose_series.poses]),
        np.asarray([pose.quaternion for pose in pose_series.poses]),
    )
    point_times = float(frame_time) + points[:, 0] * 1e-9
    relative = imu_rotations.relative_rotations(point_times, float(reference_time))
    point_positions, _ = navigation.sample_many(
        point_times + float(navigation_time_offset_s)
    )
    reference_position, reference_quaternion = navigation.sample_many(
        np.array([float(reference_time) + float(navigation_time_offset_s)])
    )

    rotation_world_body_reference = Rotation.from_quat(reference_quaternion[0]).as_matrix()
    rotation_world_lidar_reference = rotation_world_body_reference @ rotation_body_lidar
    rotation_world_imu_reference = rotation_world_lidar_reference @ rotation_imu_lidar.T
    rotation_world_imu = np.einsum(
        "ij,njk->nik", rotation_world_imu_reference, relative.as_matrix()
    )
    rotation_world_body = np.einsum(
        "nij,jk,kl->nil",
        rotation_world_imu,
        rotation_imu_lidar,
        rotation_body_lidar.T,
    )

    lidar_origins_world = point_positions + np.einsum(
        "nij,j->ni", rotation_world_body, translation_body_lidar
    )
    imu_origins_world = lidar_origins_world - np.einsum(
        "nij,j->ni", rotation_world_imu, translation_imu_lidar
    )
    points_imu = (rotation_imu_lidar @ points[:, 1:4].T).T + translation_imu_lidar
    points_world = imu_origins_world + np.einsum(
        "nij,nj->ni", rotation_world_imu, points_imu
    )

    lidar_origin_reference_world = reference_position[0] + (
        rotation_world_body_reference @ translation_body_lidar
    )
    points_lidar_reference = (
        rotation_world_lidar_reference.T
        @ (points_world - lidar_origin_reference_world).T
    ).T
    return np.column_stack([points_lidar_reference, points[:, 4]])
