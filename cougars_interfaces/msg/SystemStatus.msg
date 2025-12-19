#header
std_msgs/Header header
#0 = good, 1 = not publishing
std_msgs/Int8 depth_status
#imu status
std_msgs/Bool imu_published
#0=good, bit 1 = publishing?, bit 2 = good enough gps fix?
std_msgs/Int8 gps_status
#0 = good, 1= not publishing
std_msgs/Int8 modem_status
#0=good, bit 1=publishing?, bit 2= standard dev below threshold
std_msgs/Int8 dvl_status
#0= ok, 1 = surfacing, 2=surfaced/disarmed
std_msgs/Int8 emergency_status
