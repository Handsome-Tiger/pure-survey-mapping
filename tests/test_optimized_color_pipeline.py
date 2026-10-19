from pathlib import Path

import numpy as np
import pytest

from scripts.build_optimized_color_cloud import (
    color_output_paths,
    lidar_reference_time,
    validate_color_config,
)


def make_color_config(root):
    return {
        "outputs": {"root": str(root)},
        "mapping": {
            "deskew_orientation_source": "livox_imu",
            "extrinsic": {
                "rotation_body_lidar": np.eye(3).tolist(),
                "translation_body_lidar_m": [0.0, 0.0, 0.0],
            },
        },
        "time_alignment": {"allow_fixed_fallback": False},
        "camera": {},
    }


def test_color_output_paths_are_distinct(tmp_path):
    config = make_color_config(tmp_path)

    complete, strict = color_output_paths(config)

    assert complete.name == "cloud_optimized_color_complete.ply"
    assert strict.name == "cloud_optimized_color_strict.ply"
    assert complete != strict


def test_full_mode_rejects_active_frame_limits(tmp_path):
    config = make_color_config(tmp_path)
    config["camera"]["full_bag_required"] = True
    config["mapping"]["max_lidar_frames"] = 120

    with pytest.raises(ValueError, match="full-bag colorization forbids"):
        validate_color_config(config)


def test_validation_rejects_duplicate_paths_and_navigation_deskew(tmp_path):
    config = make_color_config(tmp_path)
    duplicate = tmp_path / "same.ply"
    config["camera"]["complete_output"] = str(duplicate)
    config["camera"]["strict_output"] = str(duplicate)
    with pytest.raises(ValueError, match="different"):
        validate_color_config(config)

    config = make_color_config(tmp_path)
    config["mapping"]["deskew_orientation_source"] = "navigation"
    with pytest.raises(ValueError, match="livox_imu"):
        validate_color_config(config)


def test_full_mode_requires_measured_time_alignment(tmp_path):
    config = make_color_config(tmp_path)
    config["camera"]["full_bag_required"] = True
    config["time_alignment"]["allow_fixed_fallback"] = True

    with pytest.raises(ValueError, match="measured time alignment"):
        validate_color_config(config)


def test_reference_time_uses_maximum_offset_not_record_order():
    offsets_ns = np.array([20_000_000.0, 5_000_000.0, 10_000_000.0])

    assert lidar_reference_time(100.0, offsets_ns) == pytest.approx(100.02)

