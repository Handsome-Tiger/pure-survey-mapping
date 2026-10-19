import calendar
import struct

from survey_mapping.time_sync_rosraw import (
    parse_fc_time_utc,
    parse_gps_utc,
    parse_ros_string,
    utc_text_to_unix,
)


def _pack_ros_string(value):
    encoded = value.encode("utf-8")
    return struct.pack("<I", len(encoded)) + encoded


def test_parse_gps_utc_reads_ros_time_and_utc_text():
    raw = struct.pack("<II", 1698133916, 946199919) + _pack_ros_string(
        "UTC 231024 075158 5 "
    )

    message = parse_gps_utc(raw)

    assert message.stamp == 1698133916.946199919
    assert message.utc_text == "UTC 231024 075158 5 "
    assert utc_text_to_unix(message.utc_text) == calendar.timegm((2023, 10, 24, 7, 51, 58))


def test_parse_fc_time_utc_reads_clock_fields():
    raw = struct.pack("<IIIII", 1698133916, 445654953, 345564085, 231024, 75157)

    message = parse_fc_time_utc(raw)

    assert message.stamp == 1698133916.445654953
    assert message.fc_timestamp_us == 345564085
    assert message.utc_yymmdd == 231024
    assert message.utc_hhmmss == 75157


def test_parse_ros_string_reads_pps_source():
    assert parse_ros_string(_pack_ros_string("RTK")) == "RTK"
