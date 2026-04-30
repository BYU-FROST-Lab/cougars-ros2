# MissionFeedback.msg

std_msgs/Header header

# --- Mission identity ---
string mission_id

# --- Mission state ---
uint8 STATE_IDLE      = 0
uint8 STATE_RUNNING   = 1
uint8 STATE_PAUSED    = 2
uint8 STATE_COMPLETE  = 3
uint8 STATE_ABORTED   = 4
uint8 state

# --- Progress (cheap to compute, cheap to transmit) ---
uint16 waypoints_completed
uint16 waypoints_total
float32 elapsed_time        # seconds since mission start

# --- Current waypoint detail ---
WaypointFeedback current

# --- Full UUID lists only if you need them for UI/logging ---
# Consider whether these belong here or in a separate latched topic
# that only publishes on waypoint transitions
unique_identifier_msgs/UUID[] completed
unique_identifier_msgs/UUID[] remaining
unique_identifier_msgs/UUID[] skipped
