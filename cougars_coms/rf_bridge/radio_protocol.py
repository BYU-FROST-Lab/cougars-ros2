# radio_protocol.py

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import ClassVar

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

    _STRUCT = struct.Struct("<14fB3f12s3Bf")

    def pack(self) -> bytes:
        return self.pack_header() + self._STRUCT.pack(
            self.x,
            self.y,
            self.depth,
            self.orientation_x,
            self.orientation_y,
            self.orientation_z,
            self.orientation_w,
            self.pressure,
            self.battery_voltage,
            self.battery_current,
            self.dvl_velocity_x,
            self.dvl_velocity_y,
            self.dvl_velocity_z,
            self.dvl_altitude,
            self.waypoint_state,
            self.horizontal_distance_error,
            self.depth_error,
            self.bearing_error,
            self.mission_id.encode("utf-8")[:12].ljust(12, b"\x00"),
            self.mission_state,
            self.waypoints_completed,
            self.waypoints_total,
            self.elapsed_time
        )

    @classmethod
    def unpack(cls, data: bytes) -> "StatusResponseMessage":
        _, src_id = cls.unpack_header(data)
        offset = cls._HEADER_STRUCT.size
        values = cls._STRUCT.unpack(data[offset : offset + cls._STRUCT.size])
        return cls(
            src_id=src_id,
            x=values[0],
            y=values[1],
            depth=values[2],
            orientation_x=values[3],
            orientation_y=values[4],
            orientation_z=values[5],
            orientation_w=values[6],
            pressure=values[7],
            battery_voltage=values[8],
            battery_current=values[9],
            dvl_velocity_x=values[10],
            dvl_velocity_y=values[11],
            dvl_velocity_z=values[12],
            dvl_altitude=values[13],
            waypoint_state=values[14],
            horizontal_distance_error=values[15],
            depth_error=values[16],
            bearing_error=values[17],
            mission_id=values[18].split(b"\x00", 1)[0].decode("utf-8", errors="ignore"),
            mission_state=values[19],
            waypoints_completed=values[20],
            waypoints_total=values[21],
            elapsed_time=values[22]
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
