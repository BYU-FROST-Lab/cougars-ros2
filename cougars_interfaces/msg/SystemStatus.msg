#header
std_msgs/Header header
#0 = good, 1 = not publishing
std_msgs/Int8 depth_status
#true = imu/data (output of the imu source relay) is publishing
std_msgs/Bool imu_published
#0=good, bit 0 (1) = gps/odom not publishing, bit 1 (2) = too few satellites, bit 2 (4) = origin likely wrong
std_msgs/Int8 gps_status
#0 = good, 1 = modem_status not publishing
std_msgs/Int8 modem_status
#0=good, bit 0 (1) = dvl/position not publishing, bit 1 (2) = position std dev above threshold
std_msgs/Int8 dvl_status
#0= ok, 1 = surfacing, 2=surfaced/disarmed
std_msgs/Int8 emergency_status
