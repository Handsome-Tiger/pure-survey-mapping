import numpy as np


def project_lidar_points(
    points_lidar,
    rotation_camera_lidar,
    translation_camera_lidar,
    intrinsics,
    distortion,
    image_shape,
    minimum_depth_m=0.2,
):
    points_lidar = np.asarray(points_lidar, dtype=float)
    rotation = np.asarray(rotation_camera_lidar, dtype=float)
    translation = np.asarray(translation_camera_lidar, dtype=float)
    intrinsics = np.asarray(intrinsics, dtype=float)
    distortion = np.pad(np.asarray(distortion, dtype=float), (0, 4), mode="constant")[:4]
    points_camera = (rotation @ points_lidar.T).T + translation
    depth = points_camera[:, 2]
    uv = np.zeros((len(points_lidar), 2), dtype=float)
    valid = depth > float(minimum_depth_m)
    if not np.any(valid):
        return uv, depth, valid

    x = points_camera[valid, 0] / depth[valid]
    y = points_camera[valid, 1] / depth[valid]
    k1, k2, p1, p2 = distortion
    radius2 = x * x + y * y
    radial = 1.0 + k1 * radius2 + k2 * radius2 * radius2
    distorted_x = x * radial + 2.0 * p1 * x * y + p2 * (radius2 + 2.0 * x * x)
    distorted_y = y * radial + p1 * (radius2 + 2.0 * y * y) + 2.0 * p2 * x * y
    u = intrinsics[0, 0] * distorted_x + intrinsics[0, 2]
    v = intrinsics[1, 1] * distorted_y + intrinsics[1, 2]
    height, width = image_shape[:2]
    inside = (u >= 0.0) & (u <= width - 1.0) & (v >= 0.0) & (v <= height - 1.0)
    indices = np.flatnonzero(valid)
    valid[:] = False
    valid[indices[inside]] = True
    uv[indices, 0] = u
    uv[indices, 1] = v
    return uv, depth, valid


def visibility_mask(uv, depth, valid, image_shape, depth_tolerance_m=0.3):
    uv = np.asarray(uv, dtype=float)
    depth = np.asarray(depth, dtype=float)
    valid = np.asarray(valid, dtype=bool)
    visible = np.zeros(len(depth), dtype=bool)
    indices = np.flatnonzero(valid)
    if not len(indices):
        return visible
    height, width = image_shape[:2]
    pixels = np.floor(uv[indices]).astype(np.int64)
    flat_pixels = pixels[:, 1] * int(width) + pixels[:, 0]
    minimum_depth = np.full(int(height) * int(width), np.inf, dtype=float)
    np.minimum.at(minimum_depth, flat_pixels, depth[indices])
    visible[indices] = depth[indices] <= minimum_depth[flat_pixels] + float(depth_tolerance_m)
    return visible
