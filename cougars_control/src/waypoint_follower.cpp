
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
#include "cougars_interfaces/msg/system_control.hpp" // For system start/stop messages
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h" // For YAML parsing

// Define a structure to hold waypoint information
struct Waypoint {
  int id;
  double enu_x;
  double enu_y;
  double depth; // Target depth at this waypoint
};

// Define a structure for LLA coordinates (primarily for origin, if needed
// later)
struct LLA {
  double latitude;
  double longitude;
  double altitude;
};

class WaypointFollowerNode : public rclcpp::Node {
public:
  WaypointFollowerNode()
      : Node("waypoint_follower"), current_waypoint_index_(0),
        mission_loaded_(false), mission_active_(false) {
    // Declare and get parameters
    this->declare_parameter<std::string>("mission_file_path", "mission.yaml");
    this->declare_parameter<double>("slip_radius", 2.0); // meters
    this->declare_parameter<double>("desired_travel_speed",
                                    20.0);              // Non-dimensional
    this->declare_parameter<double>("loop_rate", 10.0); // Hz
    mission_file_path_ = this->get_parameter("mission_file_path").as_string();
    slip_radius_ = this->get_parameter("slip_radius").as_double();
    desired_travel_speed_ =
        this->get_parameter("desired_travel_speed").as_double();
    loop_rate_ = this->get_parameter("loop_rate").as_double();

    RCLCPP_INFO(this->get_logger(), "Waypoint file path: %s",
                mission_file_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Slip radius: %.2f m", slip_radius_);
    RCLCPP_INFO(this->get_logger(), "Desired travel speed: %.2f",
                desired_travel_speed_);

    // Publishers
    desired_depth_pub_ =
        this->create_publisher<cougars_interfaces::msg::DesiredDepth>(
            "desired_depth", 10);
    desired_heading_pub_ =
        this->create_publisher<cougars_interfaces::msg::DesiredHeading>(
            "desired_heading", 10);
    desired_speed_pub_ =
        this->create_publisher<cougars_interfaces::msg::DesiredSpeed>(
            "desired_speed", 10);

    // Subscribers

    system_control_sub_ =
        this->create_subscription<cougars_interfaces::msg::SystemControl>(
            "system/status", 1,
            std::bind(&WaypointFollowerNode::system_callback, this,
                      std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "smoothed_output", 10,
        std::bind(&WaypointFollowerNode::odom_callback, this,
                  std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "modem_imu", 10,
        std::bind(&WaypointFollowerNode::imu_callback, this,
                  std::placeholders::_1));

    // Load waypoints but do not start the mission
    if (load_waypoints()) {
      mission_loaded_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Mission loaded with %zu waypoints. Waiting for system start "
                  "command.",
                  waypoints_.size());
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to load mission from %s. Node is inactive.",
                   mission_file_path_.c_str());
    }

    // Main control loop timer
    control_loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate_)),
        std::bind(&WaypointFollowerNode::control_loop_callback, this));

    // Initialize state variables
    current_x_ = 0.0;
    current_y_ = 0.0;
    current_heading_degrees_ = 0.0;
  }

private:
  // Callback for system control messages to start or stop the mission
  void
  system_callback(const cougars_interfaces::msg::SystemControl::SharedPtr msg) {
    // Handle the START command
    if (msg->start.data) {
      if (!mission_loaded_) {
        RCLCPP_WARN(
            this->get_logger(),
            "Received START command, but mission is not loaded. Ignoring.");
        return;
      }
      if (mission_active_) {
        RCLCPP_INFO(
            this->get_logger(),
            "Received START command, but mission is already active. Ignoring.");
        return;
      }

      // Check if the mission_type parameter is 'waypoint'
      if (mission_yaml_["mission_type"] &&
          mission_yaml_["mission_type"].as<std::string>() == "waypoint") {
        RCLCPP_INFO(this->get_logger(),
                    "Received START command for 'waypoint' mission. Activating "
                    "waypoint follower.");
        mission_active_ = true;
        current_waypoint_index_ = 0; // Reset to the first waypoint
      } else {
        std::string type = "undefined";
        if (mission_yaml_["mission_type"]) {
          type = mission_yaml_["mission_type"].as<std::string>();
        }
        RCLCPP_WARN(this->get_logger(),
                    "Received START command, but mission_type is '%s', not "
                    "'waypoint'. Node will not activate.",
                    type.c_str());
      }
    }
    // Handle the STOP command
    else {
      if (mission_active_) {
        RCLCPP_INFO(this->get_logger(),
                    "Received STOP command. Deactivating waypoint follower.");
        mission_active_ = false;
      }
    }
  }

  // Callback for odometry data (current position)
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
  }

  // Callback for IMU data (current heading)
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                      msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // Roll, Pitch, Yaw in radians

    // Convert yaw from radians to degrees for the controller
    current_heading_degrees_ = normalize_angle_degrees(yaw * 180.0 / M_PI);
  }

  // Main control loop
  void control_loop_callback() {
    // State 1: Mission not loaded or waypoints are empty. Hold position.
    if (!mission_loaded_ || waypoints_.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Mission not loaded or is empty. Holding position.");
      publish_control_commands(current_heading_degrees_, 0.5,
                               0.0); // Default hold depth, zero speed
      return;
    }

    // State 2: Mission loaded, but not active (waiting for start command or has
    // been stopped). Hold position.
    if (!mission_active_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Mission is inactive. Waiting for start command.");
      // Hold the depth of the first waypoint.
      publish_control_commands(current_heading_degrees_,
                               waypoints_.front().depth, 0.0);
      return;
    }

    // State 3: Mission complete. Deactivate and hold at the final waypoint's
    // depth.
    if (current_waypoint_index_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(),
                  "All waypoints reached. Mission complete. Deactivating.");
      mission_active_ = false; // It will enter State 2 in the next loop
      publish_control_commands(current_heading_degrees_,
                               waypoints_.back().depth, 0.0);
      return;
    }

    // State 4: Mission is active and running. Navigate to the current waypoint.
    const auto &target_wp = waypoints_[current_waypoint_index_];
    double distance_to_target = calculate_distance(
        current_x_, current_y_, target_wp.enu_x, target_wp.enu_y);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Active: Pos(%.2f, %.2f) -> WP %d(%.2f, %.2f) | Dist: "
                         "%.2f m | Slip Radius: %.2f m",
                         current_x_, current_y_, target_wp.id, target_wp.enu_x,
                         target_wp.enu_y, distance_to_target, slip_radius_);

    // Check if the waypoint has been reached
    if (distance_to_target < slip_radius_) {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %d. Moving to next.",
                  target_wp.id);
      current_waypoint_index_++;
      // Exit and wait for the next timer tick to process the new waypoint or
      // mission completion
      return;
    }

    // If not at the waypoint yet, calculate and publish control commands
    double desired_heading_rad = calculate_bearing(
        current_x_, current_y_, target_wp.enu_x, target_wp.enu_y);
    double desired_heading_deg =
        normalize_angle_degrees(desired_heading_rad * 180.0 / M_PI);

    publish_control_commands(desired_heading_deg, target_wp.depth,
                             desired_travel_speed_);
    RCLCPP_DEBUG(this->get_logger(),
                 "Publishing: Desired Heading: %.2f deg, Desired Depth: %.2f "
                 "m, Desired Speed: %.2f",
                 desired_heading_deg, target_wp.depth, desired_travel_speed_);
  }

  // Publish control commands
  void publish_control_commands(double heading_deg, double depth,
                                double speed) {
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
  bool load_waypoints() {
    try {
      // Use the member variable to store the parsed YAML for access in other
      // functions
      mission_yaml_ = YAML::LoadFile(mission_file_path_);

      // Add checks for all required sections for robust loading
      if (!mission_yaml_["mission_type"] || !mission_yaml_["origin_lla"] ||
          !mission_yaml_["waypoints"]) {
        RCLCPP_ERROR(this->get_logger(),
                     "YAML file missing 'mission_type', 'origin_lla', or "
                     "'waypoints' section.");
        return false;
      }

      // The original logic for parsing waypoints is preserved below
      const YAML::Node &wps = mission_yaml_["waypoints"];
      if (!wps.IsSequence()) {
        RCLCPP_ERROR(this->get_logger(),
                     "'waypoints' is not a sequence in YAML file.");
        return false;
      }

      waypoints_.clear();
      for (const auto &wp_node : wps) {
        Waypoint wp;
        wp.id = wp_node["id"].as<int>();
        wp.enu_x = wp_node["position_enu"]["x"].as<double>();
        wp.enu_y = wp_node["position_enu"]["y"].as<double>();
        wp.depth = wp_node["depth"].as<double>();
        waypoints_.push_back(wp);
        RCLCPP_INFO(this->get_logger(),
                    "Loaded WP ID: %d, ENU_X: %.2f, ENU_Y: %.2f, Depth: %.2f",
                    wp.id, wp.enu_x, wp.enu_y, wp.depth);
      }
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "YAML parsing error in %s: %s",
                   mission_file_path_.c_str(), e.what());
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error loading waypoints: %s", e.what());
      return false;
    }
    return !waypoints_.empty();
  }

  // Calculate 2D Euclidean distance
  double calculate_distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
  }

  // Calculate bearing (angle) from (x1, y1) to (x2, y2) in radians (ENU: 0
  // East, PI/2 North)
  double calculate_bearing(double x1, double y1, double x2, double y2) {
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
  rclcpp::Publisher<cougars_interfaces::msg::DesiredDepth>::SharedPtr
      desired_depth_pub_;
  rclcpp::Publisher<cougars_interfaces::msg::DesiredHeading>::SharedPtr
      desired_heading_pub_;
  rclcpp::Publisher<cougars_interfaces::msg::DesiredSpeed>::SharedPtr
      desired_speed_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<cougars_interfaces::msg::SystemControl>::SharedPtr
      system_control_sub_;
  rclcpp::TimerBase::SharedPtr control_loop_timer_;

  // Parameters
  std::string mission_file_path_;
  double slip_radius_;
  double desired_travel_speed_;
  double loop_rate_;

  // Mission State
  std::vector<Waypoint> waypoints_;
  LLA origin_lla_;
  size_t current_waypoint_index_;
  bool mission_loaded_;
  bool mission_active_;

  // Vehicle State
  double current_x_;
  double current_y_;
  double current_heading_degrees_; // ENU, 0 is East, -180 to 180
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointFollowerNode>());
  rclcpp::shutdown();
  return 0;
}