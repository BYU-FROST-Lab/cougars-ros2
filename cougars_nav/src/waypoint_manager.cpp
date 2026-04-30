
#include "rclcpp/rclcpp.hpp"

class WaypointManager : public rclcpp::Node
{
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointManager>()); 
    rclcpp::shutdown();
    return 0;
}