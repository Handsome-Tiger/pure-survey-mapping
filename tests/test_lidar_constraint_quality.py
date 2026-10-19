from survey_mapping.lidar_odometry import RegistrationResult
from scripts.build_lidar_constraints import quality_rejection_reasons


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
