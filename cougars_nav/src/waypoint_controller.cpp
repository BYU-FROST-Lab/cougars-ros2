
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "cougars_interfaces/msg/control_command.hpp"
#include "cougars_interfaces/msg/waypoint_feedback.hpp"
#include "geographic_msgs/msg/way_point.hpp"

class WaypointController : public rclcpp::Node
{
public:
    WaypointController() : Node("waypoint_follower") {
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointController>()); 
    rclcpp::shutdown();
    return 0;
}