# WaypointFeedback.msg
# All angles in radians (ENU convention), distances in meters

std_msgs/Header header

unique_identifier_msgs/UUID waypoint_id   # ID of the current target waypoint

# --- Waypoint state ---
uint8 STATE_TRANSITING  = 0   # moving toward waypoint
uint8 STATE_ARRIVED     = 1   # within acceptance radius
uint8 STATE_PARKING     = 2   # loitering at a park waypoint
uint8 STATE_SKIPPED     = 3   # waypoint unreachable, skipped
uint8 state

# --- Error metrics ---
float64 horizontal_distance_error   # meters, distance to waypoint in XY plane
float64 depth_error                 # meters, positive = too shallow
float64 bearing_error               # degrees (-180, 180), signed: positive = waypoint is to port