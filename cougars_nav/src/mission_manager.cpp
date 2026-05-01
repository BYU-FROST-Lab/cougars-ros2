
#include "rclcpp/rclcpp.hpp"
#include <memory>

class MissionManager : public rclcpp::Node
{
public:
    MissionManager() : Node("waypoint_follower") {
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionManager>()); 
    rclcpp::shutdown();
    return 0;
}