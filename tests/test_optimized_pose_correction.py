import numpy as np
from scipy.spatial.transform import Rotation

from survey_mapping.optimized_mapping import TransformCorrectionInterpolator


def test_correction_matrices_interpolate_translation_and_rotation():
    first = np.eye(4)
    second = np.eye(4)
    second[:3, :3] = Rotation.from_euler("z", 90.0, degrees=True).as_matrix()
    second[:3, 3] = [2.0, 0.0, 0.0]
    interpolator = TransformCorrectionInterpolator([0.0, 2.0], [first, second])

    matrix = interpolator.matrices_at(np.array([1.0]))[0]

    assert np.allclose(matrix[:3, 3], [1.0, 0.0, 0.0])
    assert np.allclose(
        matrix[:3, :3],
        Rotation.from_euler("z", 45.0, degrees=True).as_matrix(),
        atol=2e-3,
    )


def test_correct_poses_left_composes_world_correction():
    correction = np.eye(4)
    correction[:3, :3] = Rotation.from_euler("z", 90.0, degrees=True).as_matrix()
    correction[:3, 3] = [10.0, 0.0, 0.0]
    interpolator = TransformCorrectionInterpolator([0.0, 1.0], [correction, correction])

    position, quaternion = interpolator.correct_poses(
        np.array([[1.0, 2.0, 3.0]]),
        np.array([[0.0, 0.0, 0.0, 1.0]]),
        np.array([0.5]),
    )

    assert np.allclose(position, [[8.0, 1.0, 3.0]])
    assert np.allclose(
        Rotation.from_quat(quaternion).as_matrix()[0],
        correction[:3, :3],
    )
