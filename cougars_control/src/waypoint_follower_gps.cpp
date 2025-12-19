#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "cougars_interfaces/msg/desired_depth.hpp"
#include "cougars_interfaces/msg/desired_heading.hpp"
#include "cougars_interfaces/msg/desired_speed.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "yaml-cpp/yaml.h"

// Define a structure to hold waypoint information
struct Waypoint {
    int id;
    double enu_x; // Corresponds to 'east' or 'x' in YAML
    double enu_y; // Corresponds to 'north' or 'y' in YAML
    double depth; // Target depth at this waypoint
};

class WaypointFollowerGpsNode : public rclcpp::Node
{
public:
    WaypointFollowerGpsNode() : Node("waypoint_follower_gps"), current_waypoint_index_(0), mission_loaded_(false), mission_active_(false)
    {
        // Declare parameters
        this->declare_parameter<std::string>("waypoint_file_path", "mission.yaml");
        this->declare_parameter<double>("slip_radius", 2.0);
        this->declare_parameter<double>("desired_travel_speed", 20.0);
        this->declare_parameter<double>("loop_rate", 10.0);

        // Get parameters
        waypoint_file_path_ = this->get_parameter("waypoint_file_path").as_string();
        slip_radius_ = this->get_parameter("slip_radius").as_double();
        desired_travel_speed_ = this->get_parameter("desired_travel_speed").as_double();
        loop_rate_ = this->get_parameter("loop_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "GPS Waypoint Follower Starting...");
        RCLCPP_INFO(this->get_logger(), " - Waypoint file: %s", waypoint_file_path_.c_str());
        RCLCPP_INFO(this->get_logger(), " - Slip radius: %.2f m", slip_radius_);
        RCLCPP_INFO(this->get_logger(), " - Desired speed: %.2f", desired_travel_speed_);
        RCLCPP_INFO(this->get_logger(), " - Using /gps_odom for position and /modem_imu for heading.");

        // Publishers
        desired_depth_pub_ = this->create_publisher<cougars_interfaces::msg::DesiredDepth>("desired_depth", 10);
        desired_heading_pub_ = this->create_publisher<cougars_interfaces::msg::DesiredHeading>("desired_heading", 10);
        desired_speed_pub_ = this->create_publisher<cougars_interfaces::msg::DesiredSpeed>("desired_speed", 10);

        // Subscribers (Only GPS Odom and IMU)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "gps_odom", 10, std::bind(&WaypointFollowerGpsNode::odom_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "modem_imu", 10, std::bind(&WaypointFollowerGpsNode::imu_callback, this, std::placeholders::_1));

        // Load waypoints
        if (load_waypoints()) {
            mission_loaded_ = true;
            mission_active_ = true;
            RCLCPP_INFO(this->get_logger(), "Mission loaded successfully with %zu waypoints.", waypoints_.size());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load mission. Node will not navigate.");
        }

        // Main control loop timer
        control_loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate_)),
            std::bind(&WaypointFollowerGpsNode::control_loop_callback, this));

        current_x_ = 0.0;
        current_y_ = 0.0;
        current_heading_degrees_ = 0.0;
    }

private:
    // Callback for odometry data (gets X and Y from GPS)
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        // RCLCPP_DEBUG(this->get_logger(), "GPS Odom Received: X=%.2f, Y=%.2f", current_x_, current_y_);
    }

    // Callback for IMU data (gets heading)
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // Yaw in radians

        // Convert yaw to degrees ENU (-180 to 180, 0 is East)
        current_heading_degrees_ = normalize_angle_degrees(yaw * 180.0 / M_PI);
        // RCLCPP_DEBUG(this->get_logger(), "Current Heading (Degrees ENU): %.2f", current_heading_degrees_);
    }

    // Main control loop
    void control_loop_callback()
    {
        if (!mission_loaded_ || !mission_active_ || waypoints_.empty()) {
            // If mission isn't running, hold position (zero speed)
            publish_control_commands(current_heading_degrees_, waypoints_.empty() ? 0.5 : waypoints_.back().depth, 0.0);
            return;
        }

        // Check if we've passed the last waypoint
        if (current_waypoint_index_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "All waypoints reached. Mission complete.");
            mission_active_ = false;
            publish_control_commands(current_heading_degrees_, waypoints_.back().depth, 0.0); // Stop
            return;
        }

        const auto& target_wp = waypoints_[current_waypoint_index_];
        double distance_to_target = calculate_distance(current_x_, current_y_, target_wp.enu_x, target_wp.enu_y);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "GPS: Pos(%.2f, %.2f) | Target WP %d (%.2f, %.2f) | Dist: %.2f | Head: %.1f",
                       current_x_, current_y_, target_wp.id, target_wp.enu_x, target_wp.enu_y, distance_to_target, current_heading_degrees_);

        // Check if current waypoint is reached
        if (distance_to_target < slip_radius_) {
            RCLCPP_INFO(this->get_logger(), "Reached waypoint %d. Moving to next.", target_wp.id);
            current_waypoint_index_++;
            // If that was the last one, the check at the start of the loop will handle it next time.
            return; // Re-evaluate on the next cycle
        }

        // Calculate and publish commands for the current target waypoint
        const auto& next_target_wp = waypoints_[current_waypoint_index_];
        double desired_heading_rad = calculate_bearing(current_x_, current_y_, next_target_wp.enu_x, next_target_wp.enu_y);
        double desired_heading_deg = normalize_angle_degrees(desired_heading_rad * 180.0 / M_PI);

        publish_control_commands(desired_heading_deg, next_target_wp.depth, desired_travel_speed_);
    }

    // Publish control commands
    void publish_control_commands(double heading_deg, double depth, double speed)
    {
        auto depth_msg = cougars_interfaces::msg::DesiredDepth();
        depth_msg.desired_depth = static_cast<float>(depth);
        desired_depth_pub_->publish(depth_msg);

        auto heading_msg = cougars_interfaces::msg::DesiredHeading();
        heading_msg.desired_heading = static_cast<float>(heading_deg);
        desired_heading_pub_->publish(heading_msg);

        auto speed_msg = cougars_interfaces::msg::DesiredSpeed();
        speed_msg.desired_speed = static_cast<float>(speed);
        desired_speed_pub_->publish(speed_msg);
    }

    // Load waypoints from YAML file (Simplified)
    bool load_waypoints()
    {
        std::ifstream test_file(waypoint_file_path_);
        if (!test_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "FAILED TO OPEN YAML FILE: %s", waypoint_file_path_.c_str());
            return false;
        }
        test_file.close();

        try {
            YAML::Node mission_yaml = YAML::LoadFile(waypoint_file_path_);

            if (!mission_yaml["waypoints"]) {
                RCLCPP_ERROR(this->get_logger(), "YAML file missing 'waypoints' section.");
                return false;
            }

            const YAML::Node& wps = mission_yaml["waypoints"];
            if (!wps.IsSequence()) {
                RCLCPP_ERROR(this->get_logger(), "'waypoints' is not a sequence in YAML file.");
                return false;
            }

            waypoints_.clear();
            for (const auto& wp_node : wps) {
                Waypoint wp;
                wp.id = wp_node["id"].as<int>();
                // Try 'east'/'north' first, fall back to 'x'/'y' for flexibility
                if (wp_node["position_enu"]["east"]) {
                    wp.enu_x = wp_node["position_enu"]["east"].as<double>();
                    wp.enu_y = wp_node["position_enu"]["north"].as<double>();
                } else {
                    wp.enu_x = wp_node["position_enu"]["x"].as<double>();
                    wp.enu_y = wp_node["position_enu"]["y"].as<double>();
                }
                wp.depth = wp_node["depth"].as<double>();
                waypoints_.push_back(wp);
                RCLCPP_INFO(this->get_logger(), "Loaded WP ID: %d, ENU_X: %.2f, ENU_Y: %.2f, Depth: %.2f",
                            wp.id, wp.enu_x, wp.enu_y, wp.depth);
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
            return false;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading waypoints: %s", e.what());
            return false;
        }
        return !waypoints_.empty();
    }

    // Calculate 2D Euclidean distance
    double calculate_distance(double x1, double y1, double x2, double y2)
    {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    // Calculate bearing (angle) from (x1, y1) to (x2, y2) in radians (ENU)
    double calculate_bearing(double x1, double y1, double x2, double y2)
    {
        return std::atan2(y2 - y1, x2 - x1);
    }

    // Normalize angle to be within -180 to 180 degrees
    double normalize_angle_degrees(double angle_deg) {
        angle_deg = fmod(angle_deg + 180.0, 360.0);
        if (angle_deg < 0) {
            angle_deg += 360.0;
        }
        return angle_deg - 180.0;
    }

    // Member variables
    rclcpp::Publisher<cougars_interfaces::msg::DesiredDepth>::SharedPtr desired_depth_pub_;
    rclcpp::Publisher<cougars_interfaces::msg::DesiredHeading>::SharedPtr desired_heading_pub_;
    rclcpp::Publisher<cougars_interfaces::msg::DesiredSpeed>::SharedPtr desired_speed_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    std::string waypoint_file_path_;
    double slip_radius_;
    double desired_travel_speed_;
    double loop_rate_;

    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_;
    bool mission_loaded_;
    bool mission_active_;

    double current_x_;
    double current_y_;
    double current_heading_degrees_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointFollowerGpsNode>());
    rclcpp::shutdown();
    return 0;
}