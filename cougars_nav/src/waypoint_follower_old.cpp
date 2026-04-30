
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
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "cougars_interfaces/msg/desired_depth.hpp"
#include "cougars_interfaces/msg/desired_heading.hpp"
#include "cougars_interfaces/msg/desired_speed.hpp"
#include "cougars_interfaces/msg/system_control.hpp" // For system start/stop messages
#include "cougars_interfaces/msg/way_point.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "yaml-cpp/yaml.h" // For YAML parsing

// Define a structure to hold waypoint information
struct Waypoint {
    int id;
    double enu_x;
    double enu_y;
    double depth; // Target depth at this waypoint
};

// Define a structure for LLA coordinates (primarily for origin, if needed later)
struct LLA {
    double latitude;
    double longitude;
    double altitude;
};

class WaypointFollowerNode : public rclcpp::Node
{
public:
    WaypointFollowerNode() : Node("waypoint_follower"), current_waypoint_index_(0), mission_loaded_(false), mission_active_(false), waypoint_captured_(false)
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("mission_file_path", "mission.yaml");
        this->declare_parameter<double>("slip_radius", 2.0); // meters
        this->declare_parameter<double>("capture_radius", 10.0); // meters
        this->declare_parameter<double>("desired_travel_speed", 20.0); // Non-dimensional
        this->declare_parameter<double>("loop_rate", 10.0); // Hz
        this->declare_parameter<double>("waypoint_timeout", 300.0); // seconds (5 minutes default)
        this->declare_parameter<bool>("skip_on_timeout", true); // Skip waypoint on timeout (vs abort mission)
        this->declare_parameter<bool>("use_gps_passthrough", false); // GPS-only cart test mode (depth=0, speed=0, fins active)
        mission_file_path_ = this->get_parameter("mission_file_path").as_string();
        slip_radius_ = this->get_parameter("slip_radius").as_double();
        capture_radius_ = this->get_parameter("capture_radius").as_double();
        desired_travel_speed_ = this->get_parameter("desired_travel_speed").as_double();
        loop_rate_ = this->get_parameter("loop_rate").as_double();
        waypoint_timeout_ = this->get_parameter("waypoint_timeout").as_double();
        skip_on_timeout_ = this->get_parameter("skip_on_timeout").as_bool();
        use_gps_passthrough_ = this->get_parameter("use_gps_passthrough").as_bool();

        RCLCPP_INFO(this->get_logger(), "Waypoint file path: %s", mission_file_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Slip radius: %.2f m | Capture radius: %.2f m", slip_radius_, capture_radius_);
        RCLCPP_INFO(this->get_logger(), "Desired travel speed: %.2f", desired_travel_speed_);
        RCLCPP_INFO(this->get_logger(), "Waypoint timeout: %.1f s | Skip on timeout: %s", 
                    waypoint_timeout_, skip_on_timeout_ ? "true" : "false");
        
        // Prominent GPS passthrough mode indicator
        if (use_gps_passthrough_) {
            RCLCPP_WARN(this->get_logger(), "═══════════════════════════════════════════════════════");
            RCLCPP_WARN(this->get_logger(), "🚗 GPS PASSTHROUGH MODE ENABLED (CART TEST MODE)");
            RCLCPP_WARN(this->get_logger(), "   ➤ Depth control: DISABLED (hardcoded to 0.0m)");
            RCLCPP_WARN(this->get_logger(), "   ➤ Thruster: DISABLED (speed hardcoded to 0.0)");
            RCLCPP_WARN(this->get_logger(), "   ➤ Fin control: ACTIVE (heading PID operational)");
            RCLCPP_WARN(this->get_logger(), "   ➤ Position source: GPS only (gps_odom topic)");
            RCLCPP_WARN(this->get_logger(), "═══════════════════════════════════════════════════════");
        }

        // Publishers
        desired_depth_pub_ = this->create_publisher<cougars_interfaces::msg::DesiredDepth>("desired_depth", 10);
        desired_heading_pub_ = this->create_publisher<cougars_interfaces::msg::DesiredHeading>("desired_heading", 10);
        desired_speed_pub_ = this->create_publisher<cougars_interfaces::msg::DesiredSpeed>("desired_speed", 10);
        current_waypoint_pub_ = this->create_publisher<cougars_interfaces::msg::WayPoint>("current_waypoint", 10);

        // Subscribers

        system_control_sub_ = this->create_subscription<cougars_interfaces::msg::SystemControl>(
            "system/status", 1, std::bind(&WaypointFollowerNode::system_callback, this, std::placeholders::_1));

        // Using gps_odom for position (temporary)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "gps_odom", 10, std::bind(&WaypointFollowerNode::odom_callback, this, std::placeholders::_1));
        
        // Smoothed output (commented out for now)
        // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "smoothed_output", 10, std::bind(&WaypointFollowerNode::odom_callback, this, std::placeholders::_1));
        
        // HoloOcean simulation (commented out)
        // holoocean_odom_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        //     "/holoocean/auv0/LocationSensor", 10, std::bind(&WaypointFollowerNode::holoocean_odom_callback, this, std::placeholders::_1));
        
        // Subscribe to modem_imu for heading (same topic used by coug_controls)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "modem_imu", 10, std::bind(&WaypointFollowerNode::imu_callback, this, std::placeholders::_1));

        // Load waypoints but do not start the mission
        if (load_waypoints()) {
            mission_loaded_ = true;
            RCLCPP_INFO(this->get_logger(), "Mission loaded with %zu waypoints. Waiting for system start command.", waypoints_.size());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load mission from %s. Node is inactive.", mission_file_path_.c_str());
        }

        // Main control loop timer
        control_loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate_)),
            std::bind(&WaypointFollowerNode::control_loop_callback, this));
        
        // Initialize state variables
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_heading_degrees_ = 0.0;
        waypoint_start_time_ = this->now();
    }

private:
    // Callback for system control messages to start or stop the mission
    void system_callback(const cougars_interfaces::msg::SystemControl::SharedPtr msg)
    {
        // Handle the START command
        if (msg->start.data) {
            if (!mission_loaded_) {
                RCLCPP_WARN(this->get_logger(), "Received START command, but mission is not loaded. Ignoring.");
                return;
            }
            if (mission_active_) {
                RCLCPP_INFO(this->get_logger(), "Received START command, but mission is already active. Ignoring.");
                return;
            }

            // Check if the mission_type parameter is 'waypoint'
            if (mission_yaml_["mission_type"] && mission_yaml_["mission_type"].as<std::string>() == "waypoint") {
                RCLCPP_INFO(this->get_logger(), "Received START command for 'waypoint' mission. Activating waypoint follower.");
                mission_active_ = true;
                current_waypoint_index_ = 0; // Reset to the first waypoint
                waypoint_start_time_ = this->now(); // Start timer for first waypoint
                cougars_interfaces::msg::WayPoint current_waypoint;
                current_waypoint.waypoint_num = current_waypoint_index_ + 1; // +1 because index is 0-based and we want next WP number
                current_waypoint.x = waypoints_[current_waypoint_index_].enu_x; // Placeholder, convert ENU to L
                current_waypoint.y = waypoints_[current_waypoint_index_].enu_y; // Placeholder, convert ENU to L
                current_waypoint.depth = waypoints_[current_waypoint_index_].depth; 
                current_waypoint_pub_->publish(current_waypoint);
            } else {
                std::string type = "undefined";
                if (mission_yaml_["mission_type"]) {
                    type = mission_yaml_["mission_type"].as<std::string>();
                }
                RCLCPP_WARN(this->get_logger(),
                            "Received START command, but mission_type is '%s', not 'waypoint'. Node will not activate.",
                            type.c_str());
            }
        }
        // Handle the STOP command
        else {
            if (mission_active_) {
                RCLCPP_INFO(this->get_logger(), "Received STOP command. Deactivating waypoint follower.");
                mission_active_ = false;
            }
        }
    }

    // Callback for odometry data (nav_msgs::Odometry - used by gps_odom and smoothed_output)
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                     "Position update: (%.2f, %.2f)", current_x_, current_y_);
    }

    // Callback for HoloOcean odometry data (PoseWithCovarianceStamped)
    // NOTE: HoloOcean LocationSensor doesn't publish orientation, only position
    // Heading comes from IMU callback instead
    void holoocean_odom_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                     "Position update (HoloOcean): (%.2f, %.2f)", current_x_, current_y_);
    }

    // Callback for IMU data (current heading)
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // Roll, Pitch, Yaw in radians
        
        // Convert yaw from radians to degrees for the controller
        current_heading_degrees_ = normalize_angle_degrees(yaw * 180.0 / M_PI);
    }

    // Main control loop
    void control_loop_callback()
    {
        // State 1: Mission not loaded or waypoints are empty. Hold position.
        if (!mission_loaded_ || waypoints_.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Mission not loaded or is empty. Holding position.");
            // publish_control_commands(current_heading_degrees_, 0.5, 0.0); // Default hold depth, zero speed
            return;
        }

        // State 2: Mission loaded, but not active (waiting for start command or has been stopped). Hold position.
        if (!mission_active_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Mission is inactive. Waiting for start command.");
            // Hold the depth of the first waypoint.
            publish_control_commands(current_heading_degrees_, waypoints_.front().depth, 0.0);
            return;
        }

        // State 3: Mission complete. Deactivate and hold at the final waypoint's depth.
        if (current_waypoint_index_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "✅ All waypoints reached. Mission complete. Deactivating.");
            mission_active_ = false; // It will enter State 2 in the next loop
            publish_control_commands(current_heading_degrees_, waypoints_.back().depth, 0.0);
            return;
        }
        
        // State 4: Mission is active and running. Navigate to the current waypoint.
        const auto& target_wp = waypoints_[current_waypoint_index_];
        double distance_to_target = calculate_distance(current_x_, current_y_, target_wp.enu_x, target_wp.enu_y);

        // Check for waypoint timeout
        auto current_time = this->now();
        double elapsed_time = (current_time - waypoint_start_time_).seconds();
        
        if (elapsed_time > waypoint_timeout_) {
            if (skip_on_timeout_) {
                RCLCPP_WARN(this->get_logger(), 
                           "⏱️ TIMEOUT! WP%d not reached after %.1fs (%.1fm away). Skipping to next waypoint.", 
                           target_wp.id, elapsed_time, distance_to_target);
                current_waypoint_index_++;
                waypoint_captured_ = false;
                waypoint_start_time_ = this->now(); // Reset timer for next waypoint
                
                // Publish next waypoint info if not at the end
                if (current_waypoint_index_ < waypoints_.size()) {
                    cougars_interfaces::msg::WayPoint current_waypoint;
                    current_waypoint.waypoint_num = current_waypoint_index_ + 1;
                    current_waypoint.x = waypoints_[current_waypoint_index_].enu_x;
                    current_waypoint.y = waypoints_[current_waypoint_index_].enu_y;
                    current_waypoint.depth = waypoints_[current_waypoint_index_].depth;
                    current_waypoint_pub_->publish(current_waypoint);
                }
                return;
            } else {
                RCLCPP_ERROR(this->get_logger(), 
                            "⏱️ TIMEOUT! WP%d not reached after %.1fs (%.1fm away). Aborting mission.", 
                            target_wp.id, elapsed_time, distance_to_target);
                mission_active_ = false;
                publish_control_commands(current_heading_degrees_, target_wp.depth, 0.0);
                return;
            }
        }

        // Check if waypoint is captured (within capture radius)
        if (distance_to_target < capture_radius_ && !waypoint_captured_) {
            waypoint_captured_ = true;
            RCLCPP_INFO(this->get_logger(), "✨ Captured WP%d at %.1fm", target_wp.id, distance_to_target);
        }

        // Check if the waypoint has been reached (within slip radius)
        if (distance_to_target < slip_radius_) {
            RCLCPP_INFO(this->get_logger(), "🎯 Reached WP%d | Next target: WP%zu", 
                       target_wp.id, current_waypoint_index_ + 2);
            current_waypoint_index_++;
            waypoint_captured_ = false; // Reset for next waypoint
            waypoint_start_time_ = this->now(); // Reset timer for next waypoint
            
            // Publish next waypoint info if not at the end
            if (current_waypoint_index_ < waypoints_.size()) {
                cougars_interfaces::msg::WayPoint current_waypoint;
                current_waypoint.waypoint_num = current_waypoint_index_ + 1; // +1 because index is 0-based and we want next WP number
                current_waypoint.x = waypoints_[current_waypoint_index_].enu_x; // Placeholder, convert ENU to L
                current_waypoint.y = waypoints_[current_waypoint_index_].enu_y; // Placeholder, convert ENU to L
                current_waypoint.depth = waypoints_[current_waypoint_index_].depth;
                current_waypoint_pub_->publish(current_waypoint);
            }
            // Exit and wait for the next timer tick to process the new waypoint or mission completion
            return;
        }
        
        // If not at the waypoint yet, calculate and publish control commands
        double desired_heading_rad = calculate_bearing(current_x_, current_y_, target_wp.enu_x, target_wp.enu_y);
        double desired_heading_deg = normalize_angle_degrees(desired_heading_rad * 180.0 / M_PI);
        
        publish_control_commands(desired_heading_deg, target_wp.depth, desired_travel_speed_);
        
        // Clean, compact status update with capture status
        std::string status = waypoint_captured_ ? "CAPTURED" : "seeking";
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, 
                       "WP%d [%s]: %.1fm | Hdg: %.0f°→ %.0f° (err:%.0f°) | Spd: %.0f",
                       target_wp.id, status.c_str(), distance_to_target, 
                       current_heading_degrees_, desired_heading_deg, 
                       desired_heading_deg - current_heading_degrees_,
                       desired_travel_speed_);
    }

    // Publish control commands
    void publish_control_commands(double heading_deg, double depth, double speed)
    {
        // GPS passthrough mode overrides for cart testing
        if (use_gps_passthrough_) {
            if (depth != 0.0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                    "🚗 GPS PASSTHROUGH: Ignoring waypoint depth %.2fm (using 0.0m)", depth);
            }
            depth = 0.0;  // Hardcode depth to 0 (surface/land level)
            speed = 0.0;  // Hardcode speed to 0 (disable thruster)
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "🚗 GPS PASSTHROUGH: Depth=0.0m, Speed=0.0, Heading=%.1f° (fins active)", heading_deg);
        }
        
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

    // Load waypoints from YAML file (CORE LOGIC UNCHANGED, as requested)
    bool load_waypoints()
    {
        try {
            // Use the member variable to store the parsed YAML for access in other functions
            mission_yaml_ = YAML::LoadFile(mission_file_path_);

            // Add checks for all required sections for robust loading
            if (!mission_yaml_["mission_type"] || !mission_yaml_["origin_lla"] || !mission_yaml_["waypoints"]) {
                RCLCPP_ERROR(this->get_logger(), "YAML file missing 'mission_type', 'origin_lla', or 'waypoints' section.");
                return false;
            }
            
            // The original logic for parsing waypoints is preserved below
            const YAML::Node& wps = mission_yaml_["waypoints"];
            if (!wps.IsSequence()) {
                 RCLCPP_ERROR(this->get_logger(), "'waypoints' is not a sequence in YAML file.");
                return false;
            }

            waypoints_.clear();
            for (const auto& wp_node : wps) {
                Waypoint wp;
                wp.id = wp_node["id"].as<int>();
                wp.enu_x = wp_node["position_enu"]["x"].as<double>();
                wp.enu_y = wp_node["position_enu"]["y"].as<double>();
                wp.depth = wp_node["depth"].as<double>(); 
                waypoints_.push_back(wp);
                RCLCPP_INFO(this->get_logger(), "Loaded WP ID: %d, ENU_X: %.2f, ENU_Y: %.2f, Depth: %.2f",
                            wp.id, wp.enu_x, wp.enu_y, wp.depth);
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML parsing error in %s: %s", mission_file_path_.c_str(), e.what());
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

    // Calculate bearing (angle) from (x1, y1) to (x2, y2) in radians (ENU: 0 East, PI/2 North)
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
    YAML::Node mission_yaml_;
    rclcpp::Publisher<cougars_interfaces::msg::DesiredDepth>::SharedPtr desired_depth_pub_;
    rclcpp::Publisher<cougars_interfaces::msg::DesiredHeading>::SharedPtr desired_heading_pub_;
    rclcpp::Publisher<cougars_interfaces::msg::DesiredSpeed>::SharedPtr desired_speed_pub_;
    rclcpp::Publisher<cougars_interfaces::msg::WayPoint>::SharedPtr current_waypoint_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr holoocean_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<cougars_interfaces::msg::SystemControl>::SharedPtr system_control_sub_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    // Parameters
    std::string mission_file_path_;
    double slip_radius_;
    double capture_radius_;
    double desired_travel_speed_;
    double loop_rate_;
    double waypoint_timeout_;
    bool skip_on_timeout_;
    bool use_gps_passthrough_; // GPS-only cart test mode flag

    // Mission State
    std::vector<Waypoint> waypoints_;
    LLA origin_lla_;
    size_t current_waypoint_index_;
    bool mission_loaded_;
    bool mission_active_;
    bool waypoint_captured_;
    rclcpp::Time waypoint_start_time_; // Track when we started pursuing current waypoint

    // Vehicle State
    double current_x_;
    double current_y_;
    double current_heading_degrees_; // ENU, 0 is East, -180 to 180
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointFollowerNode>()); 
    rclcpp::shutdown();
    return 0;
}