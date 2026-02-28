
#ifndef _COUGARS_COMS_PROTOCOL_
#define _COUGARS_COMS_PROTOCOL_

#include <cstdint>

namespace cougars_coms {


enum COUG_MSG_ID : uint8_t {
    EMPTY = 0x00,

    VEHICLE_STATUS = 0x10,
    REQUEST_STATUS = 0x11,

    INIT = 0x20,        
    CONFIRM_INIT = 0x21,

    EMERGENCY_KILL = 0x40,
    CONFIRM_EMERGENCY_KILL = 0x41,

    EMERGENCY_SURFACE = 0x50,
    CONFIRM_EMERGENCY_SURFACE = 0x51,

    REQUEST_LOCALIZATION_INFO = 0x60,
    LOCALIZATION_INFO = 0x61,

};

struct RequestStatus {
    COUG_MSG_ID msg_id = REQUEST_STATUS;
}__attribute__((packed));

struct VehicleStatus {
    COUG_MSG_ID msg_id = VEHICLE_STATUS;

    uint8_t waypoint;

    float battery_voltage;
    int8_t battery_percentage;

    uint8_t depth;

    uint8_t safety_mask;

    float x;
    float y;
    int16_t x_vel;
    int16_t y_vel;
    int16_t z_vel;
    uint8_t pressure;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;

}__attribute__((packed));

struct Init {
    COUG_MSG_ID msg_id = INIT;
    uint8_t init_bitmask; // Bitmask: 0x01 = Start, 0x02 = Rosbag, 0x04 = Thruster Arm, 0x08 = DVL Acoustics
    char rosbag_prefix[28];
}__attribute__((packed));

struct ConfirmInit {
    COUG_MSG_ID msg_id = CONFIRM_INIT;
    bool success;
}__attribute__((packed));

struct EmergencyKill {
    COUG_MSG_ID msg_id = EMERGENCY_KILL;
}__attribute__((packed));

struct ConfirmEmergencyKill {
    COUG_MSG_ID msg_id = CONFIRM_EMERGENCY_KILL;
    bool success;
}__attribute__((packed));

struct EmergencySurface {
    COUG_MSG_ID msg = EMERGENCY_SURFACE;
}__attribute__((packed));

struct ConfirmEmergencySurface {
    COUG_MSG_ID msg_id = CONFIRM_EMERGENCY_SURFACE;
    bool success;
}__attribute__((packed));

struct RequestLocalizationInfo{
   COUG_MSG_ID msg_id = REQUEST_LOCALIZATION_INFO;
}__attribute__((packed));


struct LocalizationInfo {
   COUG_MSG_ID msg_id = LOCALIZATION_INFO;


   float x;
   float y;
   float z;
   float roll;
   float pitch;
   float yaw;
   float depth;
}__attribute__((packed));




} // cougars_coms
#endif //_COUGARS_COMS_PROTOCOL_

