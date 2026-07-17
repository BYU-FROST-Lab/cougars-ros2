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

    _STRUCT = struct.Struct("<BBB28sBB")

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
    x: float
    y: float
    depth: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float
    pressure: float
    battery_voltage: float
    battery_current: float
    dvl_velocity_x: float
    dvl_velocity_y: float
    dvl_velocity_z: float
    dvl_altitude: float
    waypoint_state: int
    horizontal_distance_error: float
    depth_error: float
    bearing_error: float
    mission_id: int
    mission_state: int
    waypoints_completed: int
    waypoints_total: int
    elapsed_time: float

    _STRUCT = struct.Struct("<9fB3f4Bf")

    def pack(self) -> bytes:
        return self.pack_header() + self._STRUCT.pack(
            self.x,
            self.y,
            self.depth,
            self.pressure,
            self.battery_voltage,
            self.battery_current,
            self.dvl_velocity_x,
            self.dvl_velocity_y,
            self.dvl_velocity_z,
            self.waypoint_state,
            self.horizontal_distance_error,
            self.depth_error,
            self.bearing_error,
            self.mission_id,
            self.mission_state,
            self.waypoints_completed,
            self.waypoints_total,
            self.elapsed_time
        )

    @classmethod
    def unpack(cls, data: bytes) -> "StatusResponseMessage":
        values = cls._STRUCT.unpack(data[: cls._STRUCT.size])
        return cls(
            x=values[0],
            y=values[1],
            depth=values[2],
            pressure=values[3],
            battery_voltage=values[4],
            battery_current=values[5],
            dvl_velocity_x=values[6],
            dvl_velocity_y=values[7],
            dvl_velocity_z=values[8],
            waypoint_state=values[9],
            horizontal_distance_error=values[10],
            depth_error=values[11],
            bearing_error=values[12],
            mission_id=values[13],
            mission_state=values[14],
            waypoints_completed=values[15],
            waypoints_total=values[16],
            elapsed_time=values[17]
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
