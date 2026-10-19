from dataclasses import dataclass
import struct

import numpy as np


@dataclass
class Header:
    seq: int
    stamp: float
    frame_id: str


@dataclass
class NavSatFix:
    stamp: float
    latitude: float
    longitude: float
    altitude: float
    status: int
    service: int


@dataclass
class Imu:
    stamp: float
    orientation: np.ndarray
    angular_velocity: np.ndarray
    linear_acceleration: np.ndarray


@dataclass
class QuaternionStamped:
    stamp: float
    quaternion: np.ndarray

@dataclass
class Vector3Stamped:
    stamp: float
    frame_id: str
    vector: np.ndarray


@dataclass
class LivoxCustom:
    stamp: float
    timebase_ns: int
    points: np.ndarray


@dataclass
class CompressedImage:
    stamp: float
    frame_id: str
    format: str
    data: bytes


def _unpack(fmt, data, offset):
    size = struct.calcsize(fmt)
    return struct.unpack_from(fmt, data, offset), offset + size


def parse_header(data, offset=0):
    (seq, secs, nsecs), offset = _unpack("<III", data, offset)
    (length,), offset = _unpack("<I", data, offset)
    frame_id = data[offset : offset + length].decode("utf-8")
    offset += length
    return Header(seq=seq, stamp=secs + nsecs * 1e-9, frame_id=frame_id), offset


def parse_navsatfix(data):
    header, offset = parse_header(data)
    (status, service), offset = _unpack("<bH", data, offset)
    (lat, lon, alt), offset = _unpack("<ddd", data, offset)
    return NavSatFix(
        stamp=header.stamp,
        latitude=lat,
        longitude=lon,
        altitude=alt,
        status=status,
        service=service,
    )


def parse_imu(data):
    header, offset = parse_header(data)
    orientation, offset = _unpack("<4d", data, offset)
    _, offset = _unpack("<9d", data, offset)
    angular_velocity, offset = _unpack("<3d", data, offset)
    _, offset = _unpack("<9d", data, offset)
    linear_acceleration, offset = _unpack("<3d", data, offset)
    return Imu(
        stamp=header.stamp,
        orientation=np.asarray(orientation, dtype=float),
        angular_velocity=np.asarray(angular_velocity, dtype=float),
        linear_acceleration=np.asarray(linear_acceleration, dtype=float),
    )


def parse_quaternion_stamped(data):
    header, offset = parse_header(data)
    quat, offset = _unpack("<4d", data, offset)
    return QuaternionStamped(stamp=header.stamp, quaternion=np.asarray(quat, dtype=float))


def parse_vector3_stamped(data):
    header, offset = parse_header(data)
    vector, offset = _unpack("<3d", data, offset)
    return Vector3Stamped(stamp=header.stamp, frame_id=header.frame_id, vector=np.asarray(vector, dtype=float))


def parse_int16(data):
    (value,), _offset = _unpack("<h", data, 0)
    return value


def parse_uint8(data):
    (value,), _offset = _unpack("<B", data, 0)
    return value

def parse_livox_custom(data):
    header, offset = parse_header(data)
    (timebase_ns, point_num, _lidar_id, _r0, _r1, _r2), offset = _unpack(
        "<QIB3B", data, offset
    )
    (array_len,), offset = _unpack("<I", data, offset)
    point_dtype = np.dtype(
        [
            ("offset_time", "<u4"),
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("reflectivity", "u1"),
            ("tag", "u1"),
            ("line", "u1"),
        ]
    )
    available = max(0, (len(data) - offset) // point_dtype.itemsize)
    count = min(int(point_num), int(array_len), int(available))
    records = np.frombuffer(data, dtype=point_dtype, count=count, offset=offset)
    points = np.column_stack([records[name] for name in point_dtype.names]).astype(float, copy=False)
    return LivoxCustom(stamp=header.stamp, timebase_ns=timebase_ns, points=points)


def parse_compressed_image(data):
    header, offset = parse_header(data)
    (format_len,), offset = _unpack("<I", data, offset)
    image_format = data[offset : offset + format_len].decode("utf-8", errors="replace")
    offset += format_len
    (data_len,), offset = _unpack("<I", data, offset)
    image_data = bytes(data[offset : offset + data_len])
    return CompressedImage(
        stamp=header.stamp,
        frame_id=header.frame_id,
        format=image_format,
        data=image_data,
    )
