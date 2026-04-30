# ControlCommand.msg

std_msgs/Header header

# --- Setpoints ---
float64 heading          # radians, ENU convention (0=East, pi/2=North)
float64 depth            # meters, positive down
float64 speed            # m/s, surge (forward)

# --- Validity mask: which fields the controller should act on ---
bool heading_valid
bool depth_valid
bool speed_valid