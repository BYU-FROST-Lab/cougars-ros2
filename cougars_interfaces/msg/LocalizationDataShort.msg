std_msgs/Header header

uint8 vehicle_id    # id of beacon pinged

float32 range       # range from vehicle to beacon. Divide by 10 to get meteres
float32 azimuth     # azimuth from vehicle to beacon. 360 degrees around z axis clockwise. Divide by 10 to get degrees
float32 elevation   # 90 degrees is straght up z axis, -90 is straight down. Divide by 10 to get degrees
float32 depth       # divide by 10 to get meters
