
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "cougars_interfaces/msg/waypoint_goal.hpp"
#include "cougars_interfaces/msg/waypoint_feedback.hpp"
#include "cougars_interfaces/msg/system_control.hpp"
#include "yaml-cpp/yaml.h"

struct PlannerWaypoint {
    int id;
    double enu_x;
    double enu_y;
    double depth;
    uint8_t waypoint_type;   // 0=PROXIMITY, 1=PARKING
    uint8_t depth_type;      // 0=SURFACE, 1=BOTTOM
    float speed;             // negative means "use default_speed param"
    float parking_radius;
    float parking_duration;
};

class StaticPlannerNode : public rclcpp::Node
{
public:
    StaticPlannerNode()
    : Node("static_planner"),
      current_waypoint_index_(0),
      mission_loaded_(false),
      mission_active_(false)
    {
        this->declare_parameter<std::string>("mission_file_path", "mission.yaml");
        this->declare_parameter<double>("default_speed", 20.0);
        this->declare_parameter<double>("loop_rate", 10.0);
        this->declare_parameter<double>("waypoint_timeout", 300.0);
        this->declare_parameter<bool>("skip_on_timeout", true);

        mission_file_path_ = this->get_parameter("mission_file_path").as_string();
        default_speed_ = this->get_parameter("default_speed").as_double();
        loop_rate_ = this->get_parameter("loop_rate").as_double();
        waypoint_timeout_ = this->get_parameter("waypoint_timeout").as_double();
        skip_on_timeout_ = this->get_parameter("skip_on_timeout").as_bool();

        RCLCPP_INFO(this->get_logger(), "Mission file: %s", mission_file_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Default speed: %.1f | Timeout: %.1fs | Skip on timeout: %s",
                    default_speed_, waypoint_timeout_, skip_on_timeout_ ? "true" : "false");

        waypoint_goal_pub_ = this->create_publisher<cougars_interfaces::msg::WaypointGoal>(
            "waypoint_goal", 10);

        feedback_sub_ = this->create_subscription<cougars_interfaces::msg::WaypointFeedback>(
            "waypoint_feedback", 10,
            std::bind(&StaticPlannerNode::feedback_callback, this, std::placeholders::_1));

        system_control_sub_ = this->create_subscription<cougars_interfaces::msg::SystemControl>(
            "system/status", 1,
            std::bind(&StaticPlannerNode::system_callback, this, std::placeholders::_1));

        if (load_mission()) {
            mission_loaded_ = true;
            RCLCPP_INFO(this->get_logger(), "Mission loaded with %zu waypoints. Waiting for start.",
                        waypoints_.size());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load mission from %s. Node inactive.",
                         mission_file_path_.c_str());
        }

        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate_)),
            std::bind(&StaticPlannerNode::timer_callback, this));

        waypoint_start_time_ = this->now();
    }

private:
    void system_callback(const cougars_interfaces::msg::SystemControl::SharedPtr msg)
    {
        if (msg->start.data) {
            if (!mission_loaded_) {
                RCLCPP_WARN(this->get_logger(), "START received but mission not loaded.");
                return;
            }
            if (mission_active_) {
                RCLCPP_INFO(this->get_logger(), "START received but already active.");
                return;
            }

            std::string mission_type = "undefined";
            if (mission_yaml_["mission_type"]) {
                mission_type = mission_yaml_["mission_type"].as<std::string>();
            }

            if (mission_type == "waypoint") {
                RCLCPP_INFO(this->get_logger(), "Starting waypoint mission.");
                mission_active_ = true;
                current_waypoint_index_ = 0;
                waypoint_start_time_ = this->now();
                publish_current_waypoint();
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "Mission type is '%s', not 'waypoint'. Not activating.",
                            mission_type.c_str());
            }
        } else {
            if (mission_active_) {
                RCLCPP_INFO(this->get_logger(), "STOP received. Deactivating.");
                mission_active_ = false;
            }
        }
    }

    void feedback_callback(const cougars_interfaces::msg::WaypointFeedback::SharedPtr msg)
    {
        if (!mission_active_ || current_waypoint_index_ >= waypoints_.size()) return;

        const auto& wp = waypoints_[current_waypoint_index_];
        if (msg->waypoint_id != static_cast<uint16_t>(wp.id)) return;

        bool should_advance = false;

        if (wp.waypoint_type == cougars_interfaces::msg::WaypointGoal::PROXIMITY) {
            if (msg->achieved) {
                RCLCPP_INFO(this->get_logger(), "WP%d achieved (proximity, dist=%.1fm). Advancing.",
                            wp.id, msg->distance_to_waypoint);
                should_advance = true;
            }
        } else if (wp.waypoint_type == cougars_interfaces::msg::WaypointGoal::PARKING) {
            if (msg->achieved && msg->time_in_radius >= wp.parking_duration) {
                RCLCPP_INFO(this->get_logger(),
                            "WP%d parking complete (%.1fs >= %.1fs). Advancing.",
                            wp.id, msg->time_in_radius, wp.parking_duration);
                should_advance = true;
            }
        }

        if (should_advance) {
            advance_waypoint();
        }
    }

    void timer_callback()
    {
        if (!mission_loaded_ || !mission_active_) return;

        if (current_waypoint_index_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "All waypoints completed. Mission done.");
            mission_active_ = false;
            return;
        }

        double elapsed = (this->now() - waypoint_start_time_).seconds();
        if (elapsed > waypoint_timeout_) {
            const auto& wp = waypoints_[current_waypoint_index_];
            if (skip_on_timeout_) {
                RCLCPP_WARN(this->get_logger(), "TIMEOUT on WP%d after %.1fs. Skipping.",
                            wp.id, elapsed);
                advance_waypoint();
            } else {
                RCLCPP_ERROR(this->get_logger(), "TIMEOUT on WP%d after %.1fs. Aborting mission.",
                             wp.id, elapsed);
                mission_active_ = false;
            }
            return;
        }

        publish_current_waypoint();
    }

    void advance_waypoint()
    {
        current_waypoint_index_++;
        waypoint_start_time_ = this->now();
        if (current_waypoint_index_ < waypoints_.size()) {
            const auto& wp = waypoints_[current_waypoint_index_];
            RCLCPP_INFO(this->get_logger(), "Advancing to WP%d (%zu/%zu)",
                        wp.id, current_waypoint_index_ + 1, waypoints_.size());
            publish_current_waypoint();
        }
    }

    void publish_current_waypoint()
    {
        if (current_waypoint_index_ >= waypoints_.size()) return;

        const auto& wp = waypoints_[current_waypoint_index_];
        auto msg = cougars_interfaces::msg::WaypointGoal();
        msg.header.stamp = this->now();
        msg.waypoint_id = static_cast<uint16_t>(wp.id);
        msg.pose.position.x = wp.enu_x;
        msg.pose.position.y = wp.enu_y;
        msg.pose.position.z = wp.depth;
        msg.pose.orientation.w = 1.0;
        msg.waypoint_type = wp.waypoint_type;
        msg.depth_type = wp.depth_type;
        msg.speed = (wp.speed >= 0) ? wp.speed : static_cast<float>(default_speed_);

        waypoint_goal_pub_->publish(msg);
    }

    bool load_mission()
    {
        try {
            mission_yaml_ = YAML::LoadFile(mission_file_path_);

            if (!mission_yaml_["waypoints"]) {
                RCLCPP_ERROR(this->get_logger(), "No 'waypoints' section in mission file.");
                return false;
            }

            const YAML::Node& wps = mission_yaml_["waypoints"];
            if (!wps.IsSequence()) {
                RCLCPP_ERROR(this->get_logger(), "'waypoints' is not a sequence.");
                return false;
            }

            waypoints_.clear();
            for (const auto& wp_node : wps) {
                PlannerWaypoint wp;
                wp.id = wp_node["id"].as<int>();
                wp.enu_x = wp_node["position_enu"]["x"].as<double>();
                wp.enu_y = wp_node["position_enu"]["y"].as<double>();
                wp.depth = wp_node["depth"].as<double>();

                std::string wt_str = wp_node["waypoint_type"]
                    ? wp_node["waypoint_type"].as<std::string>() : "proximity";
                wp.waypoint_type = (wt_str == "parking")
                    ? cougars_interfaces::msg::WaypointGoal::PARKING
                    : cougars_interfaces::msg::WaypointGoal::PROXIMITY;

                std::string dt_str = wp_node["depth_type"]
                    ? wp_node["depth_type"].as<std::string>() : "surface";
                wp.depth_type = (dt_str == "bottom")
                    ? cougars_interfaces::msg::WaypointGoal::DEPTH_FROM_BOTTOM
                    : cougars_interfaces::msg::WaypointGoal::DEPTH_FROM_SURFACE;

                wp.speed = wp_node["speed"] ? wp_node["speed"].as<float>() : -1.0f;
                wp.parking_radius = wp_node["parking_radius"]
                    ? wp_node["parking_radius"].as<float>() : 5.0f;
                wp.parking_duration = wp_node["parking_duration"]
                    ? wp_node["parking_duration"].as<float>() : 60.0f;

                waypoints_.push_back(wp);

                float effective_speed = (wp.speed >= 0) ? wp.speed : static_cast<float>(default_speed_);
                RCLCPP_INFO(this->get_logger(),
                    "Loaded WP%d: (%.2f, %.2f) depth=%.2f type=%s depth_ref=%s speed=%.1f",
                    wp.id, wp.enu_x, wp.enu_y, wp.depth,
                    wt_str.c_str(), dt_str.c_str(), effective_speed);
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
            return false;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading mission: %s", e.what());
            return false;
        }
        return !waypoints_.empty();
    }

    // Members
    YAML::Node mission_yaml_;
    rclcpp::Publisher<cougars_interfaces::msg::WaypointGoal>::SharedPtr waypoint_goal_pub_;
    rclcpp::Subscription<cougars_interfaces::msg::WaypointFeedback>::SharedPtr feedback_sub_;
    rclcpp::Subscription<cougars_interfaces::msg::SystemControl>::SharedPtr system_control_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::string mission_file_path_;
    double default_speed_;
    double loop_rate_;
    double waypoint_timeout_;
    bool skip_on_timeout_;

    std::vector<PlannerWaypoint> waypoints_;
    size_t current_waypoint_index_;
    bool mission_loaded_;
    bool mission_active_;
    rclcpp::Time waypoint_start_time_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
