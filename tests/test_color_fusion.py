import numpy as np

from survey_mapping.color_fusion import (
    CameraView,
    TimeOrderedCameraWindow,
    colorize_optimized_points,
)


def make_view(time_s, bgr, position=(0.0, 0.0, 0.0)):
    image = np.zeros((3, 3, 3), dtype=np.uint8)
    image[:] = np.asarray(bgr, dtype=np.uint8)
    return CameraView(
        time_s=float(time_s),
        image_bgr=image,
        position_world_body=np.asarray(position, dtype=float),
        quaternion_world_body=np.array([0.0, 0.0, 0.0, 1.0]),
    )


def identity_calibration():
    return {
        "rotation_body_lidar": np.eye(3),
        "translation_body_lidar_m": np.zeros(3),
        "rotation_camera_lidar": np.eye(3),
        "translation_camera_lidar_m": np.zeros(3),
        "intrinsics": np.array([[1.0, 0.0, 1.0], [0.0, 1.0, 1.0], [0.0, 0.0, 1.0]]),
        "distortion": np.zeros(4),
    }


def test_camera_window_returns_nearest_bounded_candidates():
    window = TimeOrderedCameraWindow(window_s=0.12, maximum_candidates=3)
    for time_s in (9.70, 9.91, 9.99, 10.04, 10.09, 10.30):
        window.add(make_view(time_s, (0, 0, 0)))

    selected = window.candidates(10.0)

    assert [view.time_s for view in selected] == [9.99, 10.04, 9.91]


def test_colorization_applies_z_buffer_and_rgb_channel_order():
    points_world = np.array([[0.0, 0.0, 2.0], [0.0, 0.0, 4.0]])
    intensities = np.array([11.0, 22.0])

    rgb, color_valid, stats = colorize_optimized_points(
        points_world,
        intensities,
        reference_time_s=10.0,
        camera_views=[make_view(10.01, (10, 20, 30))],
        depth_tolerance_m=0.2,
        **identity_calibration(),
    )

    np.testing.assert_array_equal(rgb[0], [30, 20, 10])
    np.testing.assert_array_equal(rgb[1], [22, 22, 22])
    np.testing.assert_array_equal(color_valid, [True, False])
    assert stats["visible_point_count"] == 1


def test_nearest_camera_view_wins_for_each_point():
    points_world = np.array([[0.0, 0.0, 2.0]])
    views = [
        make_view(9.95, (0, 255, 0)),
        make_view(10.01, (0, 0, 255)),
    ]

    rgb, color_valid, stats = colorize_optimized_points(
        points_world,
        np.array([5.0]),
        reference_time_s=10.0,
        camera_views=views,
        depth_tolerance_m=0.2,
        **identity_calibration(),
    )

    np.testing.assert_array_equal(rgb[0], [255, 0, 0])
    np.testing.assert_array_equal(color_valid, [True])
    assert stats["camera_view_count"] == 2
    assert stats["colored_point_count"] == 1

