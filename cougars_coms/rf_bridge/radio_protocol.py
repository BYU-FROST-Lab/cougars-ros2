# radio_protocol.py

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import ClassVar

_INT16_MIN = -32768
_INT16_MAX = 32767
_INT32_MIN = -2147483648
_INT32_MAX = 2147483647

def pack_fixed16(value: float, scale: float) -> int:
    """Scale a float by `scale` and round to an int16, clamping out-of-range values instead of raising."""
    if value != value:  # NaN
        value = 0.0
    return int(max(_INT16_MIN, min(_INT16_MAX, round(value * scale))))

def unpack_fixed16(raw: int, scale: float) -> float:
    return raw / scale

def pack_fixed32(value: float, scale: float) -> int:
    """Scale a float by `scale` and round to an int32, clamping out-of-range values instead of raising."""
    if value != value:  # NaN
        value = 0.0
    return int(max(_INT32_MIN, min(_INT32_MAX, round(value * scale))))

def unpack_fixed32(raw: int, scale: float) -> float:
    return raw / scale

class MessageID(IntEnum):
    PING = 0x00
    SYSTEM_CONTROL = 0x01
    CONFIRM_SYSTEM_CONTROL = 0x02
    DISARM_THRUSTER = 0x03
    CONFIRM_DISARM_THRUSTER = 0x04
    REQUEST_STATUS = 0x05
    STATUS_RESPONSE = 0x06
    ORIGIN_UPDATE = 0x07
    CONFIRM_ORIGIN_UPDATE = 0x08
    SURFACE_COMMAND = 0x09
    CONFIRM_SURFACE_COMMAND = 0x0A
    MISSION_RECEIVED = 0x0C

@dataclass
class RadioMessage:
    MESSAGE_ID: ClassVar[MessageID]
    src_id: int

    def pack(self) -> bytes:
        raise NotImplementedError

    _HEADER_STRUCT = struct.Struct("<BB")

    def pack_header(self) -> bytes:
        return self._HEADER_STRUCT.pack(int(self.MESSAGE_ID), int(self.src_id))

    @classmethod
    def unpack_header(cls, data: bytes) -> tuple[int, int]:
        message_id, src_id = cls._HEADER_STRUCT.unpack(data[: cls._HEADER_STRUCT.size])
        return message_id, src_id

    @classmethod
    def unpack(cls, data: bytes) -> "RadioMessage":
        raise NotImplementedError
    
@dataclass
class PingMessage(RadioMessage):
    MESSAGE_ID: ClassVar[MessageID] = MessageID.PING

    def pack(self) -> bytes:
        return self.pack_header()

    @classmethod
    def unpack(cls, data: bytes) -> "PingMessage":
        _, src_id = cls.unpack_header(data)
        return cls(src_id=src_id)

@dataclass
class SystemControlMessage(RadioMessage):
    MESSAGE_ID: ClassVar[MessageID] = MessageID.SYSTEM_CONTROL
    start: bool
    rosbag_flag: bool
    rosbag_prefix: str
    thruster_arm: bool
    dvl_acoustics: bool

    _STRUCT = struct.Struct("<BBBB28sBB")

    def pack(self) -> bytes:
        prefix = self.rosbag_prefix.encode("utf-8")[:28].ljust(28, b"\x00")
        return self._STRUCT.pack(
            int(self.MESSAGE_ID),
            int(self.src_id),
            int(self.start),
            int(self.rosbag_flag),
            prefix,
            int(self.thruster_arm),
            int(self.dvl_acoustics),
        )

    @classmethod
    def unpack(cls, data: bytes) -> "SystemControlMessage":
        _, src_id, start, rosbag_flag, rosbag_prefix, thruster_arm, dvl_acoustics = cls._STRUCT.unpack(data[: cls._STRUCT.size])
        return cls(
            src_id=src_id,
            start=bool(start),
            rosbag_flag=bool(rosbag_flag),
            rosbag_prefix=rosbag_prefix.split(b"\x00", 1)[0].decode("utf-8", errors="ignore"),
            thruster_arm=bool(thruster_arm),
            dvl_acoustics=bool(dvl_acoustics),
        )

@dataclass
class ConfirmSystemControlMessage(RadioMessage):
    MESSAGE_ID: ClassVar[MessageID] = MessageID.CONFIRM_SYSTEM_CONTROL

    def pack(self) -> bytes:
        return self.pack_header()

    @classmethod
    def unpack(cls, data: bytes) -> "ConfirmSystemControlMessage":
        _, src_id = cls.unpack_header(data)
        return cls(src_id=src_id)

@dataclass
class DisarmThrusterMessage(RadioMessage):
    MESSAGE_ID: ClassVar[MessageID] = MessageID.DISARM_THRUSTER

    def pack(self) -> bytes:
        return self.pack_header()

    @classmethod
    def unpack(cls, data: bytes) -> "DisarmThrusterMessage":
        _, src_id = cls.unpack_header(data)
        return cls(src_id=src_id)

@dataclass
class ConfirmDisarmThrusterMessage(RadioMessage):
    MESSAGE_ID: ClassVar[MessageID] = MessageID.CONFIRM_DISARM_THRUSTER

    def pack(self) -> bytes:
        return self.pack_header()

    @classmethod
    def unpack(cls, data: bytes) -> "ConfirmDisarmThrusterMessage":
        _, src_id = cls.unpack_header(data)
        return cls(src_id=src_id)
     
@dataclass
class RequestStatusMessage(RadioMessage):
    MESSAGE_ID: ClassVar[MessageID] = MessageID.REQUEST_STATUS

    def pack(self) -> bytes:
        return self.pack_header()

    @classmethod
    def unpack(cls, data: bytes) -> "RequestStatusMessage":
        _, src_id = cls.unpack_header(data)
        return cls(src_id=src_id)
    
@dataclass
class StatusResponseMessage(RadioMessage):

    MESSAGE_ID: ClassVar[MessageID] = MessageID.STATUS_RESPONSE
    x: float = 0.0
    y: float = 0.0
    depth: float = 0.0
    orientation_x: float = 0.0
    orientation_y: float = 0.0
    orientation_z: float = 0.0
    orientation_w: float = 0.0
    pressure: float = 0.0
    battery_voltage: float = 0.0
    battery_current: float = 0.0
    dvl_velocity_x: float = 0.0
    dvl_velocity_y: float = 0.0
    dvl_velocity_z: float = 0.0
    dvl_altitude: float = 0.0
    waypoint_state: int = 0
    horizontal_distance_error: float = 0.0
    depth_error: float = 0.0
    bearing_error: float = 0.0
    mission_id: str = ""
    mission_state: int = 0
    waypoints_completed: int = 0
    waypoints_total: int = 0
    elapsed_time: float = 0.0
    cov_x: float = 0.0
    cov_y: float = 0.0
    cov_z: float = 0.0
    cov_roll: float = 0.0
    cov_pitch: float = 0.0
    cov_yaw: float = 0.0

    # Most fields are packed as scaled int16s (fixed-point) instead of floats:
    # each value is multiplied by a known scale, rounded, and clamped to the
    # int16 range, so the wire size and precision are explicit and packing
    # can never raise (unlike half-precision floats, which overflow above
    # 65504 and silently lose precision in ways that are hard to predict).
    # Fields whose range isn't reliably bounded (pressure in Pascals, mission
    # elapsed time, covariance) are kept as full 4-byte floats.
    # x, y, and horizontal_distance_error are packed as int32 (not int16) so they
    # can range across a full mission area rather than being capped at +/-327.67 m.
    _POSITION_SCALE = 100.0      # cm resolution; depth stays int16 (+/-327.67 m), x/y are int32
    _ORIENTATION_SCALE = 10000.0 # 1e-4 resolution, quaternion components are in [-1, 1]
    _ELECTRICAL_SCALE = 100.0    # centi-volt / centi-amp resolution
    _VELOCITY_SCALE = 1000.0     # mm/s resolution, +/-32.767 m/s range
    _DISTANCE_SCALE = 100.0      # cm resolution; horizontal_distance_error is int32, depth/bearing stay int16

    _STRUCT = struct.Struct("<2i5hf6hBi2h12s3Bf6f")

    def pack(self) -> bytes:
        return self.pack_header() + self._STRUCT.pack(
            pack_fixed32(self.x, self._POSITION_SCALE),
            pack_fixed32(self.y, self._POSITION_SCALE),
            pack_fixed16(self.depth, self._POSITION_SCALE),
            pack_fixed16(self.orientation_x, self._ORIENTATION_SCALE),
            pack_fixed16(self.orientation_y, self._ORIENTATION_SCALE),
            pack_fixed16(self.orientation_z, self._ORIENTATION_SCALE),
            pack_fixed16(self.orientation_w, self._ORIENTATION_SCALE),
            self.pressure,
            pack_fixed16(self.battery_voltage, self._ELECTRICAL_SCALE),
            pack_fixed16(self.battery_current, self._ELECTRICAL_SCALE),
            pack_fixed16(self.dvl_velocity_x, self._VELOCITY_SCALE),
            pack_fixed16(self.dvl_velocity_y, self._VELOCITY_SCALE),
            pack_fixed16(self.dvl_velocity_z, self._VELOCITY_SCALE),
            pack_fixed16(self.dvl_altitude, self._DISTANCE_SCALE),
            self.waypoint_state,
            pack_fixed32(self.horizontal_distance_error, self._DISTANCE_SCALE),
            pack_fixed16(self.depth_error, self._DISTANCE_SCALE),
            pack_fixed16(self.bearing_error, self._DISTANCE_SCALE),
            self.mission_id.encode("utf-8")[:12].ljust(12, b"\x00"),
            self.mission_state,
            self.waypoints_completed,
            self.waypoints_total,
            self.elapsed_time,
            self.cov_x,
            self.cov_y,
            self.cov_z,
            self.cov_roll,
            self.cov_pitch,
            self.cov_yaw
        )

    @classmethod
    def unpack(cls, data: bytes) -> "StatusResponseMessage":
        _, src_id = cls.unpack_header(data)
        offset = cls._HEADER_STRUCT.size
        values = cls._STRUCT.unpack(data[offset : offset + cls._STRUCT.size])
        return cls(
            src_id=src_id,
            x=unpack_fixed32(values[0], cls._POSITION_SCALE),
            y=unpack_fixed32(values[1], cls._POSITION_SCALE),
            depth=unpack_fixed16(values[2], cls._POSITION_SCALE),
            orientation_x=unpack_fixed16(values[3], cls._ORIENTATION_SCALE),
            orientation_y=unpack_fixed16(values[4], cls._ORIENTATION_SCALE),
            orientation_z=unpack_fixed16(values[5], cls._ORIENTATION_SCALE),
            orientation_w=unpack_fixed16(values[6], cls._ORIENTATION_SCALE),
            pressure=values[7],
            battery_voltage=unpack_fixed16(values[8], cls._ELECTRICAL_SCALE),
            battery_current=unpack_fixed16(values[9], cls._ELECTRICAL_SCALE),
            dvl_velocity_x=unpack_fixed16(values[10], cls._VELOCITY_SCALE),
            dvl_velocity_y=unpack_fixed16(values[11], cls._VELOCITY_SCALE),
            dvl_velocity_z=unpack_fixed16(values[12], cls._VELOCITY_SCALE),
            dvl_altitude=unpack_fixed16(values[13], cls._DISTANCE_SCALE),
            waypoint_state=values[14],
            horizontal_distance_error=unpack_fixed32(values[15], cls._DISTANCE_SCALE),
            depth_error=unpack_fixed16(values[16], cls._DISTANCE_SCALE),
            bearing_error=unpack_fixed16(values[17], cls._DISTANCE_SCALE),
            mission_id=values[18].split(b"\x00", 1)[0].decode("utf-8", errors="ignore"),
            mission_state=values[19],
            waypoints_completed=values[20],
            waypoints_total=values[21],
            elapsed_time=values[22],
            cov_x=values[23],
            cov_y=values[24],
            cov_z=values[25],
            cov_roll=values[26],
            cov_pitch=values[27],
            cov_yaw=values[28]
        )
    
@dataclass
class OriginUpdateMessage(RadioMessage):
    MESSAGE_ID: ClassVar[MessageID] = MessageID.ORIGIN_UPDATE
    latitude: float
    longitude: float
    altitude: float

    def pack(self) -> bytes:
        return self.pack_header() + struct.pack("<fff", self.latitude, self.longitude, self.altitude)

    @classmethod
    def unpack(cls, data: bytes) -> "OriginUpdateMessage":
        _, src_id = cls.unpack_header(data)
        latitude, longitude, altitude = struct.unpack("<fff", data[cls._HEADER_STRUCT.size : cls._HEADER_STRUCT.size + struct.calcsize("<fff")])
        return cls(src_id=src_id, latitude=latitude, longitude=longitude, altitude=altitude)
    
@dataclass
class ConfirmOriginUpdateMessage(RadioMessage):

    MESSAGE_ID: ClassVar[MessageID] = MessageID.CONFIRM_ORIGIN_UPDATE

    def pack(self) -> bytes:
        return self.pack_header()

    @classmethod
    def unpack(cls, data: bytes) -> "ConfirmOriginUpdateMessage":
        _, src_id = cls.unpack_header(data)
        return cls(src_id=src_id)
    
@dataclass
class MissionReceivedMessage(RadioMessage):
    MESSAGE_ID: ClassVar[MessageID] = MessageID.MISSION_RECEIVED

    def pack(self) -> bytes:
        return self.pack_header()

    @classmethod
    def unpack(cls, data: bytes) -> "MissionReceivedMessage":
        _, src_id = cls.unpack_header(data)
        return cls(src_id=src_id)
