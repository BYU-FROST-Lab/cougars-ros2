# VehicleSetpoint.msg

std_msgs/Header header

# --- Depth mode ---
uint8 DEPTH    = 0    # depth is meters below surface (positive down)
uint8 ALTITUDE = 1    # depth field is desired altitude above seafloor (m); transformer converts to DEPTH

uint8 mode            # DEPTH (default) or ALTITUDE

# --- Setpoints ---
float64 heading          # radians, ENU convention (0=East, pi/2=North)
float64 depth            # meters — interpretation depends on mode
float64 speed            # m/s, surge (forward)

# --- Validity mask: which fields the controller should act on ---
bool heading_valid
bool depth_valid
bool speed_valid