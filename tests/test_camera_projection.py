import numpy as np

from survey_mapping.camera import (
    colorize_body_points,
    sample_bgr_bilinear,
    intrinsics_from_fov,
    project_body_points,
)


def test_project_body_points_keeps_points_inside_image():
    k = intrinsics_from_fov(100, 80, 90.0, 60.0)
    points = np.array([[0.0, 0.0, 5.0], [10.0, 0.0, 1.0], [0.0, 0.0, -1.0]])
    uv, valid = project_body_points(points, np.eye(3), np.zeros(3), k, (80, 100, 3))

    assert valid.tolist() == [True, False, False]
    assert np.allclose(uv[0], [50, 40], atol=1)


def test_colorize_body_points_samples_rgb_and_falls_back_to_intensity():
    image = np.zeros((20, 20, 3), dtype=np.uint8)
    image[10, 10] = [30, 20, 10]  # BGR
    k = np.array([[10.0, 0.0, 10.0], [0.0, 10.0, 10.0], [0.0, 0.0, 1.0]])
    points = np.array([[0.0, 0.0, 2.0], [100.0, 0.0, 2.0]])

    rgb, valid = colorize_body_points(points, np.array([7.0, 9.0]), image, np.eye(3), np.zeros(3), k)

    assert valid.tolist() == [True, False]
    assert rgb[0].tolist() == [10, 20, 30]
    assert rgb[1].tolist() == [9, 9, 9]


def test_sample_bgr_bilinear_interpolates_between_four_pixels():
    image = np.array(
        [
            [[0, 0, 0], [10, 20, 30]],
            [[20, 40, 60], [30, 60, 90]],
        ],
        dtype=np.uint8,
    )

    sampled = sample_bgr_bilinear(image, np.array([[0.5, 0.5]]))

    assert sampled[0].tolist() == [15, 30, 45]
