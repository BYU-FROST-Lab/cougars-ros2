
#include "rclcpp/rclcpp.hpp"
#include <memory>

class WaypointManager : public rclcpp::Node
{
public:
    WaypointManager() : Node("waypoint_follower") {
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointManager>()); 
    rclcpp::shutdown();
    return 0;
}