/**
 * Waypoint Follower Node
 * 
 * This node autonomously navigates the vehicle through a sequence of GPS waypoints.
 * It subscribes to position and heading data, calculates desired heading to reach
 * each waypoint, and publishes control commands (depth, heading, speed).
 * 
 * States:
 *   1. Mission not loaded - hold position
 *   2. Mission loaded but inactive - wait for start command
 *   3. Mission active - navigate to waypoints
 *   4. Mission complete - hold at final position
 */

#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "cougars_interfaces/msg/desired_depth.hpp"
#include "cougars_interfaces/msg/desired_heading.hpp"
#include "cougars_interfaces/msg/desired_speed.hpp"
#include "cougars_interfaces/msg/system_control.hpp"
#include "cougars_interfaces/msg/way_point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"


// =============================================================================
// DATA STRUCTURES
// =============================================================================

// Single waypoint definition from mission file
struct Waypoint {
  int id;           // Waypoint number (1, 2, 3, ...)
  double enu_x;     // East coordinate in meters from origin
  double enu_y;     // North coordinate in meters from origin
  double depth;     // Target depth in meters (positive = deeper)
};

// Latitude, Longitude, Altitude - used for mission origin
struct LLA {
  double latitude;
  double longitude;
  double altitude;
};


// =============================================================================
// WAYPOINT FOLLOWER NODE CLASS
// =============================================================================

class WaypointFollowerNode : public rclcpp::Node {
public:
  WaypointFollowerNode()
      : Node("waypoint_follower"),
        current_waypoint_index_(0),
        mission_loaded_(false),
        mission_active_(false),
        waypoint_captured_(false) {
    
    // -------------------------------------------------------------------------
    // PARAMETER SETUP
    // -------------------------------------------------------------------------
    
    // Path to mission YAML file
    this->declare_parameter<std::string>("mission_file_path", "mission.yaml");
    
    // Navigation radii
    this->declare_parameter<double>("slip_radius", 2.0);      // meters - waypoint "reached" threshold
    this->declare_parameter<double>("capture_radius", 10.0);  // meters - waypoint "captured" threshold
    
    // Speed and timing
    this->declare_parameter<double>("desired_travel_speed", 20.0);  // Non-dimensional (0-100)
    this->declare_parameter<double>("loop_rate", 10.0);             // Hz - control loop frequency
    this->declare_parameter<double>("waypoint_timeout", 300.0);     // seconds
    
    // Behavior flags
    this->declare_parameter<bool>("skip_on_timeout", true);       // Skip waypoint vs abort on timeout
    this->declare_parameter<bool>("use_gps_passthrough", false);  // Cart test mode (depth=0, speed=0)
    
    // Load parameters into member variables
    mission_file_path_ = this->get_parameter("mission_file_path").as_string();
    slip_radius_ = this->get_parameter("slip_radius").as_double();
    capture_radius_ = this->get_parameter("capture_radius").as_double();
    desired_travel_speed_ = this->get_parameter("desired_travel_speed").as_double();
    loop_rate_ = this->get_parameter("loop_rate").as_double();
    waypoint_timeout_ = this->get_parameter("waypoint_timeout").as_double();
    skip_on_timeout_ = this->get_parameter("skip_on_timeout").as_bool();
    use_gps_passthrough_ = this->get_parameter("use_gps_passthrough").as_bool();
    
    // Log configuration
    RCLCPP_INFO(this->get_logger(), "Waypoint file path: %s", mission_file_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Slip radius: %.2f m | Capture radius: %.2f m", 
                slip_radius_, capture_radius_);
    RCLCPP_INFO(this->get_logger(), "Desired travel speed: %.2f", desired_travel_speed_);
    RCLCPP_INFO(this->get_logger(), "Waypoint timeout: %.1f s | Skip on timeout: %s",
                waypoint_timeout_, skip_on_timeout_ ? "true" : "false");
    
    // Warn if GPS passthrough mode is enabled (cart testing)
    if (use_gps_passthrough_) {
      RCLCPP_WARN(this->get_logger(), "═══════════════════════════════════════════════════════");
      RCLCPP_WARN(this->get_logger(), "GPS PASSTHROUGH MODE ENABLED (CART TEST MODE)");
      RCLCPP_WARN(this->get_logger(), "   Depth control: DISABLED (hardcoded to 0.0m)");
      RCLCPP_WARN(this->get_logger(), "   Thruster: DISABLED (speed hardcoded to 0.0)");
      RCLCPP_WARN(this->get_logger(), "   Fin control: ACTIVE (heading PID operational)");
      RCLCPP_WARN(this->get_logger(), "   Position source: GPS only (gps_odom topic)");
      RCLCPP_WARN(this->get_logger(), "═══════════════════════════════════════════════════════");
    }
    
    // -------------------------------------------------------------------------
    // PUBLISHERS
    // -------------------------------------------------------------------------
    
    // Publish desired depth to coug_controls
    desired_depth_pub_ = this->create_publisher<cougars_interfaces::msg::DesiredDepth>(
        "desired_depth", 10);
    
    // Publish desired heading to coug_controls
    desired_heading_pub_ = this->create_publisher<cougars_interfaces::msg::DesiredHeading>(
        "desired_heading", 10);
    
    // Publish desired speed to coug_controls
    desired_speed_pub_ = this->create_publisher<cougars_interfaces::msg::DesiredSpeed>(
        "desired_speed", 10);
    
    // Publish current waypoint info for monitoring/GUI
    current_waypoint_pub_ = this->create_publisher<cougars_interfaces::msg::WayPoint>(
        "current_waypoint", 10);
    
    // -------------------------------------------------------------------------
    // SUBSCRIBERS
    // -------------------------------------------------------------------------
    
    // Subscribe to system start/stop commands
    system_control_sub_ = this->create_subscription<cougars_interfaces::msg::SystemControl>(
        "system/status", 1,
        std::bind(&WaypointFollowerNode::system_callback, this, std::placeholders::_1));
    
    // Subscribe to GPS position (converted to ENU coordinates)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "gps_odom", 10,
        std::bind(&WaypointFollowerNode::odom_callback, this, std::placeholders::_1));
    
    // Subscribe to IMU for current heading
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "modem_imu", 10,
        std::bind(&WaypointFollowerNode::imu_callback, this, std::placeholders::_1));
    
    // -------------------------------------------------------------------------
    // INITIALIZATION
    // -------------------------------------------------------------------------
    
    // Load waypoints from mission file
    if (load_waypoints()) {
      mission_loaded_ = true;
      RCLCPP_INFO(this->get_logger(), "Mission loaded with %zu waypoints. Waiting for start command.",
                  waypoints_.size());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to load mission from %s. Node is inactive.",
                   mission_file_path_.c_str());
    }
    
    // Create main control loop timer
    int timer_period_ms = static_cast<int>(1000.0 / loop_rate_);
    control_loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms),
        std::bind(&WaypointFollowerNode::control_loop_callback, this));
    
    // Initialize vehicle state
    current_x_ = 0.0;
    current_y_ = 0.0;
    current_heading_degrees_ = 0.0;
    waypoint_start_time_ = this->now();
  }

private:
  // ===========================================================================
  // CALLBACK FUNCTIONS
  // ===========================================================================
  
  // Handle system start/stop commands
  void system_callback(const cougars_interfaces::msg::SystemControl::SharedPtr msg) {
    // START command received
    if (msg->start.data) {
      // Validate we can start
      if (!mission_loaded_) {
        RCLCPP_WARN(this->get_logger(), 
                    "Received START command, but mission is not loaded. Ignoring.");
        return;
      }
      if (mission_active_) {
        RCLCPP_INFO(this->get_logger(),
                    "Received START command, but mission is already active. Ignoring.");
        return;
      }
      
      // Only start if mission type is "waypoint"
      if (mission_yaml_["mission_type"] && 
          mission_yaml_["mission_type"].as<std::string>() == "waypoint") {
        
        RCLCPP_INFO(this->get_logger(), 
                    "Received START command for waypoint mission. Activating.");
        
        // Activate mission
        mission_active_ = true;
        current_waypoint_index_ = 0;
        waypoint_start_time_ = this->now();
        waypoint_captured_ = false;
        
        // Publish first waypoint info
        publish_current_waypoint();
        
      } else {
        std::string type = "undefined";
        if (mission_yaml_["mission_type"]) {
          type = mission_yaml_["mission_type"].as<std::string>();
        }
        RCLCPP_WARN(this->get_logger(),
                    "Received START command, but mission_type is '%s', not 'waypoint'. "
                    "Node will not activate.", type.c_str());
      }
    }
    // STOP command received
    else {
      if (mission_active_) {
        RCLCPP_INFO(this->get_logger(), "Received STOP command. Deactivating waypoint follower.");
        mission_active_ = false;
      }
    }
  }
  
  // Update current position from GPS odometry
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Position update: (%.2f, %.2f)", current_x_, current_y_);
  }
  
  // Update current heading from IMU
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Convert quaternion to roll-pitch-yaw
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                      msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // Convert yaw from radians to degrees and normalize
    current_heading_degrees_ = normalize_angle_degrees(yaw * 180.0 / M_PI);
  }
  
  // ===========================================================================
  // MAIN CONTROL LOOP
  // ===========================================================================
  
  void control_loop_callback() {
    
    // -------------------------------------------------------------------------
    // STATE 1: Mission not loaded or empty
    // -------------------------------------------------------------------------
    if (!mission_loaded_ || waypoints_.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Mission not loaded or is empty. Holding position.");
      return;
    }
    
    // -------------------------------------------------------------------------
    // STATE 2: Mission loaded but not active
    // -------------------------------------------------------------------------
    if (!mission_active_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Mission is inactive. Waiting for start command.");
      // Hold at first waypoint's depth, zero speed
      publish_control_commands(current_heading_degrees_, waypoints_.front().depth, 0.0);
      return;
    }
    
    // -------------------------------------------------------------------------
    // STATE 3: Mission complete
    // -------------------------------------------------------------------------
    if (current_waypoint_index_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "All waypoints reached. Mission complete. Deactivating.");
      mission_active_ = false;
      publish_control_commands(current_heading_degrees_, waypoints_.back().depth, 0.0);
      return;
    }
    
    // -------------------------------------------------------------------------
    // STATE 4: Mission active - navigate to current waypoint
    // -------------------------------------------------------------------------
    const Waypoint& target_wp = waypoints_[current_waypoint_index_];
    
    // Calculate distance to target
    double distance_to_target = calculate_distance(
        current_x_, current_y_, target_wp.enu_x, target_wp.enu_y);
    
    // Check for timeout
    auto current_time = this->now();
    double elapsed_time = (current_time - waypoint_start_time_).seconds();
    
    if (elapsed_time > waypoint_timeout_) {
      handle_waypoint_timeout(target_wp, distance_to_target, elapsed_time);
      return;
    }
    
    // Check if waypoint is captured (within capture radius)
    if (distance_to_target < capture_radius_ && !waypoint_captured_) {
      waypoint_captured_ = true;
      RCLCPP_INFO(this->get_logger(), "Captured WP%d at %.1fm", target_wp.id, distance_to_target);
    }
    
    // Check if waypoint is reached (within slip radius)
    if (distance_to_target < slip_radius_) {
      advance_to_next_waypoint(target_wp);
      return;
    }
    
    // Navigate to waypoint - calculate desired heading
    double desired_heading_rad = calculate_bearing(
        current_x_, current_y_, target_wp.enu_x, target_wp.enu_y);
    double desired_heading_deg = normalize_angle_degrees(desired_heading_rad * 180.0 / M_PI);
    
    // Publish control commands
    publish_control_commands(desired_heading_deg, target_wp.depth, desired_travel_speed_);
    
    // Log status periodically
    std::string status = waypoint_captured_ ? "CAPTURED" : "seeking";
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "WP%d [%s]: %.1fm | Hdg: %.0f -> %.0f (err:%.0f) | Spd: %.0f",
                         target_wp.id, status.c_str(), distance_to_target,
                         current_heading_degrees_, desired_heading_deg,
                         desired_heading_deg - current_heading_degrees_, 
                         desired_travel_speed_);
  }
  
  // ===========================================================================
  // HELPER FUNCTIONS
  // ===========================================================================
  
  // Handle waypoint timeout - either skip or abort
  void handle_waypoint_timeout(const Waypoint& target_wp, double distance, double elapsed) {
    if (skip_on_timeout_) {
      RCLCPP_WARN(this->get_logger(),
                  "TIMEOUT! WP%d not reached after %.1fs (%.1fm away). Skipping.",
                  target_wp.id, elapsed, distance);
      current_waypoint_index_++;
      waypoint_captured_ = false;
      waypoint_start_time_ = this->now();
      publish_current_waypoint();
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "TIMEOUT! WP%d not reached after %.1fs (%.1fm away). Aborting mission.",
                   target_wp.id, elapsed, distance);
      mission_active_ = false;
      publish_control_commands(current_heading_degrees_, target_wp.depth, 0.0);
    }
  }
  
  // Advance to next waypoint after reaching current one
  void advance_to_next_waypoint(const Waypoint& target_wp) {
    RCLCPP_INFO(this->get_logger(), "Reached WP%d | Next target: WP%zu",
                target_wp.id, current_waypoint_index_ + 2);
    
    current_waypoint_index_++;
    waypoint_captured_ = false;
    waypoint_start_time_ = this->now();
    
    publish_current_waypoint();
  }
  
  // Publish current waypoint info for monitoring
  void publish_current_waypoint() {
    if (current_waypoint_index_ >= waypoints_.size()) {
      return;
    }
    
    cougars_interfaces::msg::WayPoint current_waypoint;
    current_waypoint.waypoint_num = current_waypoint_index_ + 1;  // 1-based for display
    current_waypoint.x = waypoints_[current_waypoint_index_].enu_x;
    current_waypoint.y = waypoints_[current_waypoint_index_].enu_y;
    current_waypoint.depth = waypoints_[current_waypoint_index_].depth;
    current_waypoint_pub_->publish(current_waypoint);
  }
  
  // Publish control commands to coug_controls
  void publish_control_commands(double heading_deg, double depth, double speed) {
    // GPS passthrough mode overrides for cart testing
    if (use_gps_passthrough_) {
      if (depth != 0.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                             "GPS PASSTHROUGH: Ignoring waypoint depth %.2fm (using 0.0m)",
                             depth);
      }
      depth = 0.0;  // Force surface level
      speed = 0.0;  // Disable thruster
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "GPS PASSTHROUGH: Depth=0.0m, Speed=0.0, Heading=%.1f (fins active)",
                           heading_deg);
    }
    
    // Publish depth command
    auto depth_msg = cougars_interfaces::msg::DesiredDepth();
    depth_msg.desired_depth = static_cast<float>(depth);
    desired_depth_pub_->publish(depth_msg);
    
    // Publish heading command
    auto heading_msg = cougars_interfaces::msg::DesiredHeading();
    heading_msg.desired_heading = static_cast<float>(heading_deg);
    desired_heading_pub_->publish(heading_msg);
    
    // Publish speed command
    auto speed_msg = cougars_interfaces::msg::DesiredSpeed();
    speed_msg.desired_speed = static_cast<float>(speed);
    desired_speed_pub_->publish(speed_msg);
  }
  
  // Load waypoints from YAML mission file
  bool load_waypoints() {
    try {
      mission_yaml_ = YAML::LoadFile(mission_file_path_);
      
      // Validate required sections exist
      if (!mission_yaml_["mission_type"] || !mission_yaml_["origin_lla"] ||
          !mission_yaml_["waypoints"]) {
        RCLCPP_ERROR(this->get_logger(),
                     "YAML file missing 'mission_type', 'origin_lla', or 'waypoints' section.");
        return false;
      }
      
      // Validate waypoints is a sequence
      const YAML::Node& wps = mission_yaml_["waypoints"];
      if (!wps.IsSequence()) {
        RCLCPP_ERROR(this->get_logger(), "'waypoints' is not a sequence in YAML file.");
        return false;
      }
      
      // Parse each waypoint
      waypoints_.clear();
      for (const auto& wp_node : wps) {
        Waypoint wp;
        wp.id = wp_node["id"].as<int>();
        wp.enu_x = wp_node["position_enu"]["x"].as<double>();
        wp.enu_y = wp_node["position_enu"]["y"].as<double>();
        wp.depth = wp_node["depth"].as<double>();
        waypoints_.push_back(wp);
        
        RCLCPP_INFO(this->get_logger(), "Loaded WP%d: ENU(%.2f, %.2f) Depth:%.2f",
                    wp.id, wp.enu_x, wp.enu_y, wp.depth);
      }
      
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "YAML parsing error in %s: %s",
                   mission_file_path_.c_str(), e.what());
      return false;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error loading waypoints: %s", e.what());
      return false;
    }
    
    return !waypoints_.empty();
  }
  
  // Calculate 2D Euclidean distance between two points
  double calculate_distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
  }
  
  // Calculate bearing angle from (x1,y1) to (x2,y2) in radians
  // ENU frame: 0 = East, PI/2 = North
  double calculate_bearing(double x1, double y1, double x2, double y2) {
    return std::atan2(y2 - y1, x2 - x1);
  }
  
  // Normalize angle to range [-180, 180] degrees
  double normalize_angle_degrees(double angle_deg) {
    angle_deg = fmod(angle_deg + 180.0, 360.0);
    if (angle_deg < 0) {
      angle_deg += 360.0;
    }
    return angle_deg - 180.0;
  }
  
  // ===========================================================================
  // MEMBER VARIABLES
  // ===========================================================================
  
  // ROS communication
  rclcpp::Publisher<cougars_interfaces::msg::DesiredDepth>::SharedPtr desired_depth_pub_;
  rclcpp::Publisher<cougars_interfaces::msg::DesiredHeading>::SharedPtr desired_heading_pub_;
  rclcpp::Publisher<cougars_interfaces::msg::DesiredSpeed>::SharedPtr desired_speed_pub_;
  rclcpp::Publisher<cougars_interfaces::msg::WayPoint>::SharedPtr current_waypoint_pub_;
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
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
  bool use_gps_passthrough_;
  
  // Mission data
  YAML::Node mission_yaml_;
  std::vector<Waypoint> waypoints_;
  LLA origin_lla_;
  
  // Mission state
  size_t current_waypoint_index_;
  bool mission_loaded_;
  bool mission_active_;
  bool waypoint_captured_;
  rclcpp::Time waypoint_start_time_;
  
  // Vehicle state
  double current_x_;
  double current_y_;
  double current_heading_degrees_;  // ENU frame: 0 = East, range [-180, 180]
};


// =============================================================================
// MAIN ENTRY POINT
// =============================================================================

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
