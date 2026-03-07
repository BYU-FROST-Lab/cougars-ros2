# DesiredGoal: Waypoint Manager -> coug_controls
# Unified setpoint replacing DesiredDepth, DesiredHeading, DesiredSpeed

std_msgs/Header header

# Desired depth in meters (positive = deeper)
float64 desired_depth

# Desired heading in degrees, ENU frame (-180 to 180, 0 = East)
float64 desired_heading

# Desired speed (non-dimensional, 0-100)
float64 desired_speed

# Depth-from-bottom flag (true = DVL altitude mode, false = pressure depth mode)
bool dfb false
