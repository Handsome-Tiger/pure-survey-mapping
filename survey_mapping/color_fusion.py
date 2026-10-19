from bisect import bisect_right
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from survey_mapping.calibrated_camera import project_lidar_points, visibility_mask
from survey_mapping.camera import sample_bgr_bilinear


@dataclass(frozen=True)
class CameraView:
    time_s: float
    image_bgr: np.ndarray
    position_world_body: np.ndarray
    quaternion_world_body: np.ndarray


class TimeOrderedCameraWindow:
    def __init__(self, samples=None, window_s=0.12, maximum_candidates=3):
        self.window_s = float(window_s)
        self.maximum_candidates = int(maximum_candidates)
        if self.window_s < 0.0:
            raise ValueError("window_s must be non-negative")
        if self.maximum_candidates < 1:
            raise ValueError("maximum_candidates must be positive")
        self._samples = []
        for sample in samples or ():
            self.add(sample)

    def add(self, sample):
        times = [item.time_s for item in self._samples]
        index = bisect_right(times, float(sample.time_s))
        self._samples.insert(index, sample)

    def discard_before(self, minimum_time_s):
        minimum = float(minimum_time_s)
        self._samples = [item for item in self._samples if item.time_s >= minimum]

    def candidates(self, target_time_s):
        target = float(target_time_s)
        eligible = [
            item
            for item in self._samples
            if abs(float(item.time_s) - target) <= self.window_s
        ]
        eligible.sort(key=lambda item: (abs(float(item.time_s) - target), item.time_s))
        return eligible[: self.maximum_candidates]


def colorize_optimized_points(
    points_world,
    intensities,
    reference_time_s,
    camera_views,
    rotation_body_lidar,
    translation_body_lidar_m,
    rotation_camera_lidar,
    translation_camera_lidar_m,
    intrinsics,
    distortion,
    depth_tolerance_m=0.3,
):
    points_world = np.asarray(points_world, dtype=float)
    intensities = np.asarray(intensities, dtype=float)
    if points_world.ndim != 2 or points_world.shape[1] != 3:
        raise ValueError("points_world must have shape (N, 3)")
    if intensities.shape != (len(points_world),):
        raise ValueError("intensities must have shape (N,)")

    gray = np.clip(np.rint(intensities), 0, 255).astype(np.uint8)
    rgb = np.repeat(gray[:, None], 3, axis=1)
    color_valid = np.zeros(len(points_world), dtype=bool)
    r_bl = np.asarray(rotation_body_lidar, dtype=float)
    t_bl = np.asarray(translation_body_lidar_m, dtype=float)
    ordered_views = sorted(
        camera_views,
        key=lambda view: (abs(float(view.time_s) - float(reference_time_s)), view.time_s),
    )

    projected_point_count = 0
    visible_point_count = 0
    for view in ordered_views:
        position = np.asarray(view.position_world_body, dtype=float)
        quaternion = np.asarray(view.quaternion_world_body, dtype=float)
        points_body = Rotation.from_quat(quaternion).inv().apply(points_world - position)
        points_lidar = (r_bl.T @ (points_body - t_bl).T).T
        uv, depth, projected = project_lidar_points(
            points_lidar,
            rotation_camera_lidar,
            translation_camera_lidar_m,
            intrinsics,
            distortion,
            view.image_bgr.shape,
        )
        visible = visibility_mask(
            uv,
            depth,
            projected,
            view.image_bgr.shape,
            depth_tolerance_m=depth_tolerance_m,
        )
        projected_point_count += int(np.count_nonzero(projected))
        visible_point_count += int(np.count_nonzero(visible))
        assign = visible & ~color_valid
        if not np.any(assign):
            continue
        rgb[assign] = sample_bgr_bilinear(view.image_bgr, uv[assign])[:, ::-1]
        color_valid[assign] = True

    stats = {
        "camera_view_count": int(len(ordered_views)),
        "projected_point_count": int(projected_point_count),
        "visible_point_count": int(visible_point_count),
        "colored_point_count": int(np.count_nonzero(color_valid)),
    }
    return rgb, color_valid, stats

