from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def _interpolate_vectors(times, sample_times, values):
    return np.column_stack(
        [np.interp(sample_times, times, values[:, column]) for column in range(values.shape[1])]
    )


def _interpolate_quaternions(times, sample_times, quaternions):
    indices = np.searchsorted(times, sample_times, side="right")
    indices = np.clip(indices, 1, len(times) - 1)
    first = quaternions[indices - 1].copy()
    second = quaternions[indices].copy()
    flip = np.einsum("ij,ij->i", first, second) < 0.0
    second[flip] *= -1.0
    denominator = times[indices] - times[indices - 1]
    ratio = np.divide(
        sample_times - times[indices - 1],
        denominator,
        out=np.zeros_like(sample_times, dtype=float),
        where=denominator != 0.0,
    )
    ratio = np.clip(ratio, 0.0, 1.0)[:, None]
    interpolated = first * (1.0 - ratio) + second * ratio
    interpolated /= np.linalg.norm(interpolated, axis=1, keepdims=True)
    interpolated[sample_times <= times[0]] = quaternions[0]
    interpolated[sample_times >= times[-1]] = quaternions[-1]
    return interpolated


class PoseInterpolator:
    def __init__(self, times, positions, quaternions):
        self.times = np.asarray(times, dtype=float)
        self.positions = np.asarray(positions, dtype=float)
        self.quaternions = np.asarray(quaternions, dtype=float)
        if len(self.times) < 2:
            raise ValueError("PoseInterpolator requires at least two poses")

    def sample_many(self, sample_times):
        sample_times = np.asarray(sample_times, dtype=float)
        positions = _interpolate_vectors(self.times, sample_times, self.positions)
        quaternions = _interpolate_quaternions(self.times, sample_times, self.quaternions)
        return positions, quaternions

    def transform_body_points(self, points_body, sample_times):
        positions, quaternions = self.sample_many(sample_times)
        return Rotation.from_quat(quaternions).apply(points_body) + positions


class TransformCorrectionInterpolator:
    def __init__(self, times, transforms):
        self.times = np.asarray(times, dtype=float)
        transforms = np.asarray(transforms, dtype=float)
        self.translations = transforms[:, :3, 3]
        self.quaternions = Rotation.from_matrix(transforms[:, :3, :3]).as_quat()
        if len(self.times) < 2:
            raise ValueError("TransformCorrectionInterpolator requires at least two corrections")

    def sample_many(self, sample_times):
        sample_times = np.asarray(sample_times, dtype=float)
        translations = _interpolate_vectors(self.times, sample_times, self.translations)
        quaternions = _interpolate_quaternions(self.times, sample_times, self.quaternions)
        return translations, quaternions

    def matrices_at(self, sample_times):
        translations, quaternions = self.sample_many(sample_times)
        matrices = np.repeat(np.eye(4, dtype=float)[None, :, :], len(translations), axis=0)
        matrices[:, :3, :3] = Rotation.from_quat(quaternions).as_matrix()
        matrices[:, :3, 3] = translations
        return matrices

    def correct_poses(self, positions, quaternions, sample_times):
        positions = np.asarray(positions, dtype=float)
        quaternions = np.asarray(quaternions, dtype=float)
        sample_times = np.asarray(sample_times, dtype=float)
        if positions.shape != (len(sample_times), 3):
            raise ValueError("positions must have shape (N, 3)")
        if quaternions.shape != (len(sample_times), 4):
            raise ValueError("quaternions must have shape (N, 4)")
        initial = np.repeat(np.eye(4, dtype=float)[None, :, :], len(sample_times), axis=0)
        initial[:, :3, :3] = Rotation.from_quat(quaternions).as_matrix()
        initial[:, :3, 3] = positions
        corrected = self.matrices_at(sample_times) @ initial
        return corrected[:, :3, 3], Rotation.from_matrix(corrected[:, :3, :3]).as_quat()

    def apply(self, points_world, sample_times):
        translations, quaternions = self.sample_many(sample_times)
        return Rotation.from_quat(quaternions).apply(points_world) + translations


class BinaryPlyWriter:
    _COUNT_WIDTH = 20

    def __init__(self, path, with_rgb=False):
        self.path = Path(path)
        self.with_rgb = bool(with_rgb)
        self.handle = None
        self.count = 0
        self.count_offset = None

    def __enter__(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = self.path.open("w+b")
        self.handle.write(b"ply\nformat binary_little_endian 1.0\nelement vertex ")
        self.count_offset = self.handle.tell()
        self.handle.write(b"0" * self._COUNT_WIDTH)
        self.handle.write(b"\nproperty float x\nproperty float y\nproperty float z\n")
        if self.with_rgb:
            self.handle.write(b"property uchar red\nproperty uchar green\nproperty uchar blue\n")
        else:
            self.handle.write(b"property uchar intensity\n")
        self.handle.write(b"end_header\n")
        return self

    def write(self, points_xyz, attributes):
        points = np.asarray(points_xyz, dtype=np.float32)
        attributes = np.asarray(attributes, dtype=np.uint8)
        if len(points) != len(attributes):
            raise ValueError("Point and attribute counts differ")
        if self.with_rgb:
            dtype = np.dtype([("xyz", "<f4", (3,)), ("rgb", "u1", (3,))])
            records = np.empty(len(points), dtype=dtype)
            records["xyz"] = points
            records["rgb"] = attributes.reshape(-1, 3)
        else:
            dtype = np.dtype([("xyz", "<f4", (3,)), ("intensity", "u1")])
            records = np.empty(len(points), dtype=dtype)
            records["xyz"] = points
            records["intensity"] = attributes.reshape(-1)
        self.handle.write(records.tobytes())
        self.count += len(points)

    def __exit__(self, exc_type, exc_value, traceback):
        if self.handle is None:
            return
        if exc_type is None:
            self.handle.seek(self.count_offset)
            self.handle.write(f"{self.count:0{self._COUNT_WIDTH}d}".encode("ascii"))
        self.handle.close()
