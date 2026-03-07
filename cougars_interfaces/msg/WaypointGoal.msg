# WaypointGoal: Static Planner -> Waypoint Manager
# Represents the current waypoint target for navigation

std_msgs/Header header

# Current waypoint index (1-based)
uint16 waypoint_id

# Target pose (position.x=ENU_east, position.y=ENU_north, position.z=depth positive-deeper)
geometry_msgs/Pose pose

# Waypoint type
uint8 PROXIMITY=0
uint8 PARKING=1
uint8 waypoint_type

# Depth reference type
uint8 DEPTH_FROM_SURFACE=0
uint8 DEPTH_FROM_BOTTOM=1
uint8 depth_type

# Desired travel speed (non-dimensional, 0-100)
float32 speed
