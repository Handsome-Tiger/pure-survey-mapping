from survey_mapping.lidar_odometry import RegistrationResult
from scripts.build_lidar_constraints import graph_component_count, quality_limits, quality_rejection_reasons


def test_iteration_limit_is_not_a_rejection_when_measured_quality_passes():
    result = RegistrationResult(
        transform=None,
        rmse=0.25,
        inlier_fraction=0.7,
        converged=False,
        iterations=30,
    )

    reasons = quality_rejection_reasons(
        result,
        correction_translation=0.2,
        correction_rotation=1.0,
        minimum_inlier=0.25,
        maximum_rmse=1.0,
        maximum_translation=2.5,
        maximum_rotation=5.0,
    )

    assert reasons == []


def test_adjacent_edges_can_use_separate_limits_without_relaxing_strip_edges():
    registration = {
        "maximum_rmse_m": 0.8,
        "maximum_correction_m": 3.0,
        "maximum_correction_deg": 6.0,
        "adjacent_maximum_rmse_m": 0.9,
        "adjacent_maximum_correction_m": 4.0,
        "adjacent_maximum_correction_deg": 25.0,
    }

    assert quality_limits(registration, "strip")["maximum_rotation"] == 6.0
    assert quality_limits(registration, "adjacent") == {
        "minimum_inlier": 0.25,
        "maximum_rmse": 0.9,
        "maximum_translation": 4.0,
        "maximum_rotation": 25.0,
    }


def test_graph_component_count_detects_disconnected_constraints():
    assert graph_component_count(4, [(0, 1), (1, 2), (2, 3)]) == 1
    assert graph_component_count(4, [(0, 1), (2, 3)]) == 2

