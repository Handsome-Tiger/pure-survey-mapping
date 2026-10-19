import struct

import numpy as np
from scipy.spatial.transform import Rotation

from survey_mapping.optimized_mapping import (
    BinaryPlyWriter,
    PoseInterpolator,
    TransformCorrectionInterpolator,
)


def make_transform(translation, yaw_deg=0.0):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
    transform[:3, 3] = translation
    return transform


def test_pose_interpolator_batches_position_and_shortest_quaternion_path():
    poses = PoseInterpolator(
        np.array([0.0, 2.0]),
        np.array([[0.0, 0.0, 0.0], [2.0, 4.0, 0.0]]),
        Rotation.from_euler("z", [170.0, 190.0], degrees=True).as_quat(),
    )

    position, quaternion = poses.sample_many(np.array([0.0, 1.0, 2.0]))

    assert np.allclose(position[1], [1.0, 2.0, 0.0])
    midpoint_yaw = Rotation.from_quat(quaternion[1]).as_euler("zyx", degrees=True)[0]
    assert abs(abs(midpoint_yaw) - 180.0) < 1e-6


def test_transform_correction_interpolates_translation_and_rotation():
    correction = TransformCorrectionInterpolator(
        np.array([10.0, 20.0]),
        np.stack([make_transform([0.0, 0.0, 0.0]), make_transform([2.0, 0.0, 0.0], 20.0)]),
    )

    transformed = correction.apply(np.array([[1.0, 0.0, 0.0]]), np.array([15.0]))

    expected = Rotation.from_euler("z", 10.0, degrees=True).apply([[1.0, 0.0, 0.0]])[0]
    expected += [1.0, 0.0, 0.0]
    assert np.allclose(transformed[0], expected, atol=1e-6)


def test_binary_ply_writer_patches_vertex_count_and_writes_rgb(tmp_path):
    path = tmp_path / "cloud.ply"
    with BinaryPlyWriter(path, with_rgb=True) as writer:
        writer.write(
            np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]),
            np.array([[10, 20, 30], [40, 50, 60]], dtype=np.uint8),
        )

    content = path.read_bytes()
    header, payload = content.split(b"end_header\n", 1)
    assert b"element vertex 00000000000000000002" in header
    assert b"property uchar red" in header
    assert len(payload) == 2 * struct.calcsize("<fffBBB")
