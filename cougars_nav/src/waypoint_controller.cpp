
#include "rclcpp/rclcpp.hpp"
#include <memory>

class WaypointController : public rclcpp::Node
{
public:
    WaypointManager() : Node("waypoint_follower") {
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointController>()); 
    rclcpp::shutdown();
    return 0;
}