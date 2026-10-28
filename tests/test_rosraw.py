import struct

import numpy as np

from survey_mapping.rosraw import parse_header, parse_imu, parse_int16, parse_livox_custom, parse_navsatfix, parse_odometry, parse_uint8, parse_vector3_stamped


def pack_header(seq=7, secs=10, nsecs=20, frame_id="map"):
    raw = struct.pack("<III", seq, secs, nsecs)
    encoded = frame_id.encode("utf-8")
    return raw + struct.pack("<I", len(encoded)) + encoded


def test_parse_header_reads_ros_string_and_time():
    header, offset = parse_header(pack_header(frame_id="livox"))
    assert header.seq == 7
    assert header.stamp == 10.00000002
    assert header.frame_id == "livox"
    assert offset == 4 + 4 + 4 + 4 + 5


def test_parse_navsatfix_reads_lat_lon_alt():
    raw = pack_header() + struct.pack("<bH", 0, 1)
    raw += struct.pack("<ddd", 22.1, 114.2, 9.5)
    raw += struct.pack("<9dB", *([0.0] * 9), 0)
    msg = parse_navsatfix(raw)
    assert msg.stamp == 10.00000002
    assert msg.latitude == 22.1
    assert msg.longitude == 114.2
    assert msg.altitude == 9.5


def test_parse_imu_reads_angular_velocity_and_acceleration():
    raw = pack_header()
    raw += struct.pack("<4d", 0.0, 0.0, 0.0, 1.0)
    raw += struct.pack("<9d", *([0.0] * 9))
    raw += struct.pack("<3d", 1.0, 2.0, 3.0)
    raw += struct.pack("<9d", *([0.0] * 9))
    raw += struct.pack("<3d", 4.0, 5.0, 6.0)
    raw += struct.pack("<9d", *([0.0] * 9))
    msg = parse_imu(raw)
    assert np.allclose(msg.angular_velocity, [1.0, 2.0, 3.0])
    assert np.allclose(msg.linear_acceleration, [4.0, 5.0, 6.0])


def test_parse_livox_custom_reads_points():
    raw = pack_header(frame_id="livox_frame")
    raw += struct.pack("<QIB3B", 123456789, 2, 1, 0, 0, 0)
    raw += struct.pack("<I", 2)
    raw += struct.pack("<IfffBBB", 0, 1.0, 2.0, 3.0, 10, 0, 1)
    raw += struct.pack("<IfffBBB", 5000, 4.0, 5.0, 6.0, 20, 0, 2)
    msg = parse_livox_custom(raw)
    assert msg.timebase_ns == 123456789
    assert msg.points.shape == (2, 7)
    assert np.allclose(msg.points[1, :4], [5000.0, 4.0, 5.0, 6.0])


def test_parse_vector3_stamped_reads_timestamp_and_vector():
    raw = pack_header(frame_id="enu") + struct.pack("<3d", 1.25, -2.5, 3.75)

    msg = parse_vector3_stamped(raw)

    assert msg.stamp == 10.00000002
    assert msg.frame_id == "enu"
    assert np.allclose(msg.vector, [1.25, -2.5, 3.75])


def test_parse_odometry_reads_pose_and_linear_velocity():
    child = b"gnss_ins_enu"
    raw = pack_header(frame_id="gnss_ins") + struct.pack("<I", len(child)) + child
    raw += struct.pack("<3d", 1.0, 2.0, 3.0)
    raw += struct.pack("<4d", 0.1, 0.2, 0.3, 0.9)
    raw += struct.pack("<36d", *([0.0] * 36))
    raw += struct.pack("<3d", 4.0, 5.0, 6.0)
    raw += struct.pack("<3d", 0.0, 0.0, 0.0)
    raw += struct.pack("<36d", *([0.0] * 36))

    msg = parse_odometry(raw)

    assert msg.frame_id == "gnss_ins"
    assert msg.child_frame_id == "gnss_ins_enu"
    assert np.allclose(msg.position, [1.0, 2.0, 3.0])
    assert np.allclose(msg.quaternion, [0.1, 0.2, 0.3, 0.9])
    assert np.allclose(msg.linear_velocity, [4.0, 5.0, 6.0])


def test_parse_scalar_std_messages_without_fabricating_timestamps():
    assert parse_int16(struct.pack("<h", -123)) == -123
    assert parse_uint8(struct.pack("<B", 50)) == 50

