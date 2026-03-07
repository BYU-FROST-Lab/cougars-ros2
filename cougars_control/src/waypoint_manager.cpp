
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "dvl_msgs/msg/dvl.hpp"
#include "cougars_interfaces/msg/waypoint_goal.hpp"
#include "cougars_interfaces/msg/waypoint_feedback.hpp"
#include "cougars_interfaces/msg/desired_goal.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class WaypointManagerNode : public rclcpp::Node
{
public:
    WaypointManagerNode()
    : Node("waypoint_manager"),
      has_goal_(false),
      has_position_(false),
      has_heading_(false),
      current_x_(0.0),
      current_y_(0.0),
      current_heading_deg_(0.0),
      pressure_depth_(0.0),
      dvl_altitude_(0.0),
      time_in_radius_(0.0),
      was_in_radius_(false)
    {
        this->declare_parameter<double>("capture_radius", 10.0);
        this->declare_parameter<double>("slip_radius", 2.0);
        this->declare_parameter<double>("loop_rate", 10.0);
        this->declare_parameter<bool>("use_gps_passthrough", false);

        capture_radius_ = this->get_parameter("capture_radius").as_double();
        slip_radius_ = this->get_parameter("slip_radius").as_double();
        loop_rate_ = this->get_parameter("loop_rate").as_double();
        use_gps_passthrough_ = this->get_parameter("use_gps_passthrough").as_bool();

        RCLCPP_INFO(this->get_logger(), "Capture radius: %.1fm | Slip radius: %.1fm",
                    capture_radius_, slip_radius_);

        if (use_gps_passthrough_) {
            RCLCPP_WARN(this->get_logger(),
                "GPS PASSTHROUGH MODE: depth=0, speed=0, heading control active");
        }

        // Publishers
        desired_goal_pub_ = this->create_publisher<cougars_interfaces::msg::DesiredGoal>(
            "desired_goal", 10);
        feedback_pub_ = this->create_publisher<cougars_interfaces::msg::WaypointFeedback>(
            "waypoint_feedback", 10);

        // Subscribers
        waypoint_goal_sub_ = this->create_subscription<cougars_interfaces::msg::WaypointGoal>(
            "waypoint_goal", 10,
            std::bind(&WaypointManagerNode::waypoint_goal_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "gps_odom", 10,
            std::bind(&WaypointManagerNode::odom_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "modem_imu", 10,
            std::bind(&WaypointManagerNode::imu_callback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "depth_data", 10,
            std::bind(&WaypointManagerNode::depth_callback, this, std::placeholders::_1));

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
            qos_profile);

        dvl_sub_ = this->create_subscription<dvl_msgs::msg::DVL>(
            "dvl/data", qos,
            std::bind(&WaypointManagerNode::dvl_callback, this, std::placeholders::_1));

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate_)),
            std::bind(&WaypointManagerNode::control_loop, this));

        last_radius_check_time_ = this->now();
    }

private:
    void waypoint_goal_callback(const cougars_interfaces::msg::WaypointGoal::SharedPtr msg)
    {
        if (!has_goal_ || msg->waypoint_id != current_goal_.waypoint_id) {
            time_in_radius_ = 0.0;
            was_in_radius_ = false;
            last_radius_check_time_ = this->now();
            RCLCPP_INFO(this->get_logger(),
                "New waypoint goal: WP%d (%.1f, %.1f) depth=%.2f type=%d depth_ref=%d speed=%.1f",
                msg->waypoint_id,
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                msg->waypoint_type, msg->depth_type, msg->speed);
        }
        current_goal_ = *msg;
        has_goal_ = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        has_position_ = true;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x, msg->orientation.y,
            msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_heading_deg_ = normalize_angle_degrees(yaw * 180.0 / M_PI);
        has_heading_ = true;
    }

    void depth_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        pressure_depth_ = -msg->pose.pose.position.z;
    }

    void dvl_callback(const dvl_msgs::msg::DVL::SharedPtr msg)
    {
        dvl_altitude_ = msg->altitude;
    }

    void control_loop()
    {
        if (!has_goal_ || !has_position_) return;

        double target_x = current_goal_.pose.position.x;
        double target_y = current_goal_.pose.position.y;
        double target_depth = current_goal_.pose.position.z;
        double distance = calculate_distance(current_x_, current_y_, target_x, target_y);

        double desired_heading = current_heading_deg_;
        if (has_heading_) {
            double bearing_rad = std::atan2(target_y - current_y_, target_x - current_x_);
            desired_heading = normalize_angle_degrees(bearing_rad * 180.0 / M_PI);
        }

        bool dfb = (current_goal_.depth_type ==
                    cougars_interfaces::msg::WaypointGoal::DEPTH_FROM_BOTTOM);

        // Publish DesiredGoal to coug_controls
        auto goal_msg = cougars_interfaces::msg::DesiredGoal();
        goal_msg.header.stamp = this->now();

        if (use_gps_passthrough_) {
            goal_msg.desired_depth = 0.0;
            goal_msg.desired_speed = 0.0;
            goal_msg.dfb = false;
        } else {
            goal_msg.desired_depth = target_depth;
            goal_msg.desired_speed = static_cast<double>(current_goal_.speed);
            goal_msg.dfb = dfb;
        }
        goal_msg.desired_heading = desired_heading;
        desired_goal_pub_->publish(goal_msg);

        // Determine waypoint achievement for feedback
        bool achieved = false;
        float time_in_radius_report = 0.0f;

        if (current_goal_.waypoint_type == cougars_interfaces::msg::WaypointGoal::PROXIMITY) {
            achieved = (distance < slip_radius_);
        } else if (current_goal_.waypoint_type == cougars_interfaces::msg::WaypointGoal::PARKING) {
            double dt = (this->now() - last_radius_check_time_).seconds();
            last_radius_check_time_ = this->now();

            if (distance < capture_radius_) {
                time_in_radius_ += dt;
                was_in_radius_ = true;
            } else {
                if (was_in_radius_) {
                    time_in_radius_ = 0.0;
                    was_in_radius_ = false;
                }
            }
            time_in_radius_report = static_cast<float>(time_in_radius_);
            achieved = was_in_radius_;
        }

        // Publish WaypointFeedback to Static Planner
        auto fb_msg = cougars_interfaces::msg::WaypointFeedback();
        fb_msg.header.stamp = this->now();
        fb_msg.waypoint_id = current_goal_.waypoint_id;
        fb_msg.distance_to_waypoint = static_cast<float>(distance);
        fb_msg.achieved = achieved;
        fb_msg.time_in_radius = time_in_radius_report;
        feedback_pub_->publish(fb_msg);

        std::string status = achieved ? "ACHIEVED" : "seeking";
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "WP%d [%s]: dist=%.1fm hdg=%.0f->%.0f spd=%.0f depth=%.2f",
            current_goal_.waypoint_id, status.c_str(), distance,
            current_heading_deg_, desired_heading,
            goal_msg.desired_speed, target_depth);
    }

    double calculate_distance(double x1, double y1, double x2, double y2)
    {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    double normalize_angle_degrees(double angle_deg)
    {
        angle_deg = fmod(angle_deg + 180.0, 360.0);
        if (angle_deg < 0) angle_deg += 360.0;
        return angle_deg - 180.0;
    }

    // Publishers
    rclcpp::Publisher<cougars_interfaces::msg::DesiredGoal>::SharedPtr desired_goal_pub_;
    rclcpp::Publisher<cougars_interfaces::msg::WaypointFeedback>::SharedPtr feedback_pub_;

    // Subscribers
    rclcpp::Subscription<cougars_interfaces::msg::WaypointGoal>::SharedPtr waypoint_goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_sub_;
    rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr dvl_sub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    // Parameters
    double capture_radius_;
    double slip_radius_;
    double loop_rate_;
    bool use_gps_passthrough_;

    // Current waypoint goal from Static Planner
    cougars_interfaces::msg::WaypointGoal current_goal_;
    bool has_goal_;

    // Vehicle state
    bool has_position_;
    bool has_heading_;
    double current_x_;
    double current_y_;
    double current_heading_deg_;
    double pressure_depth_;
    double dvl_altitude_;

    // Parking waypoint tracking
    double time_in_radius_;
    bool was_in_radius_;
    rclcpp::Time last_radius_check_time_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointManagerNode>());
    rclcpp::shutdown();
    return 0;
}
