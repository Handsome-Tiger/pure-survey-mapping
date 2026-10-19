from scripts.run_production_pipeline import configured_topics, resolved_config


def minimal_config():
    return {
        "bag": {
            "path": "old.bag",
            "lidar_topic": "/points",
            "livox_imu_topic": "/livox_imu",
            "imu_topic": "/imu",
            "attitude_topic": "/attitude",
            "gnss_topic": "/gnss",
            "rtk_velocity_topic": "/velocity",
            "rtk_yaw_topic": "/yaw",
            "rtk_quality_topic": "/quality",
        },
        "mapping": {},
        "camera": {"topic": "/camera"},
        "outputs": {"root": "old-output"},
    }


def test_resolved_config_overrides_all_delivery_paths(tmp_path):
    config = minimal_config()
    bag = tmp_path / "flight.bag"
    output = tmp_path / "result"
    result = resolved_config(config, bag, output)

    assert result["bag"]["path"] == str(bag.resolve())
    assert result["outputs"]["root"] == str(output.resolve())
    assert result["mapping"]["optimized_cloud_output"].endswith(
        "cloud_optimized_geometry_livox_deskew.ply"
    )
    assert result["camera"]["complete_output"].endswith(
        "cloud_optimized_color_complete.ply"
    )
    assert result["camera"]["strict_output"].endswith(
        "cloud_optimized_color_strict.ply"
    )
    assert config["bag"]["path"] == "old.bag"


def test_configured_topics_follow_yaml_names():
    topics = configured_topics(minimal_config())
    assert topics["/points"] == "livox_ros_driver/CustomMsg"
    assert topics["/camera"] == "sensor_msgs/CompressedImage"
    assert len(topics) == 9

