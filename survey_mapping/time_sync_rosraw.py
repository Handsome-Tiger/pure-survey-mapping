from dataclasses import dataclass
from datetime import datetime, timezone
import struct


@dataclass(frozen=True)
class GpsUtc:
    stamp: float
    utc_text: str


@dataclass(frozen=True)
class FcTimeUtc:
    stamp: float
    fc_timestamp_us: int
    utc_yymmdd: int
    utc_hhmmss: int


def _parse_ros_time(data, offset=0):
    secs, nsecs = struct.unpack_from("<II", data, offset)
    return secs + nsecs * 1e-9, offset + 8


def parse_ros_string(data, offset=0):
    (length,) = struct.unpack_from("<I", data, offset)
    offset += 4
    end = offset + int(length)
    if end > len(data):
        raise ValueError("ROS string length exceeds message size")
    return data[offset:end].decode("utf-8", errors="replace")


def parse_gps_utc(data):
    stamp, offset = _parse_ros_time(data)
    return GpsUtc(stamp=stamp, utc_text=parse_ros_string(data, offset))


def parse_fc_time_utc(data):
    stamp, offset = _parse_ros_time(data)
    fc_timestamp_us, utc_yymmdd, utc_hhmmss = struct.unpack_from("<III", data, offset)
    return FcTimeUtc(
        stamp=stamp,
        fc_timestamp_us=int(fc_timestamp_us),
        utc_yymmdd=int(utc_yymmdd),
        utc_hhmmss=int(utc_hhmmss),
    )


def utc_text_to_unix(utc_text):
    fields = str(utc_text).strip().split()
    if len(fields) < 3 or fields[0] != "UTC":
        raise ValueError("GPS UTC text must have form 'UTC yymmdd hhmmss ...'")
    parsed = datetime.strptime(fields[1] + fields[2], "%y%m%d%H%M%S")
    return parsed.replace(tzinfo=timezone.utc).timestamp()


def fc_fields_to_unix(utc_yymmdd, utc_hhmmss):
    text = f"{int(utc_yymmdd):06d}{int(utc_hhmmss):06d}"
    parsed = datetime.strptime(text, "%y%m%d%H%M%S")
    return parsed.replace(tzinfo=timezone.utc).timestamp()
