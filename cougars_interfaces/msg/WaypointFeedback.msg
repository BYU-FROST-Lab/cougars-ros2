# WaypointFeedback: Waypoint Manager -> Static Planner
# Reports progress toward the current waypoint

std_msgs/Header header

# Waypoint being tracked
uint16 waypoint_id

# 2D distance to waypoint in meters
float32 distance_to_waypoint

# Whether the waypoint condition has been met
bool achieved

# Time spent within parking radius (seconds), relevant for parking waypoints
float32 time_in_radius
