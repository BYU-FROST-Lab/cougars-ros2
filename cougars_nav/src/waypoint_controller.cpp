
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <cmath>
#include <string>
#include <unordered_map>
#include "cougars_interfaces/msg/vehicle_setpoint.hpp"
#include "cougars_interfaces/msg/waypoint_feedback.hpp"
#include "cougars_interfaces/msg/system_control.hpp"
#include "geographic_msgs/msg/way_point.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using std::placeholders::_1;

static constexpr double EARTH_RADIUS_METERS = 6371000.0;

/*
Navigates to waypoints published on the "waypoint" topic.

Converts each waypoint's lat/lon to ENU x/y using the same Haversine formula
as gps_odom, with the origin received from the transient-local "origin" topic.
Depth comes from position.altitude (positive-down). Mode is set via the "DFB"
(depth-from-bottom) prop: "true" → ALTITUDE mode, anything else → DEPTH mode.
Speed and arrival radii come from props ("speed", "cap", "slip"), falling back
to node parameters.

Publishes VehicleSetpoint on "guidance/setpoint_raw" whenever transiting, and
WaypointFeedback on "waypoint_feedback" each control loop tick. Transitions
to STATE_ARRIVED when within slip_radius, or STATE_SKIPPED on timeout.

Subscribes:
  - "waypoint"       (geographic_msgs/WayPoint)       — from waypoint_iterator
  - "origin"         (geographic_msgs/GeoPoint)        — transient-local
  - "odometry/global" (nav_msgs/Odometry) — current pose
  - "system/control" (cougars_interfaces/SystemControl) — to know when to stop

Publishes:
  - "guidance/setpoint_raw"  (cougars_interfaces/VehicleSetpoint)
  - "waypoint_feedback" (cougars_interfaces/WaypointFeedback)

Arrival logic:
  - Capture radius (cap): if the robot enters this radius, it arrives immediately.
  - Slip radius (slip): larger than cap. If the robot enters and then exits this
    radius (having dwelled inside for at least slip_dwell_time to filter GPS noise),
    it also counts as arrived. This prevents the robot looping indefinitely when it
    overshoots the capture radius.

Parameters:
  - loop_rate             (double, Hz,      default 10.0)
  - waypoint_timeout      (double, seconds, default 300.0)
  - skip_on_timeout       (bool,            default true)
  - default_speed         (double, m/s,     default 1.0)
  - default_capture_radius (double, meters, default 2.0)
  - default_slip_radius   (double, meters,  default 10.0)
  - slip_dwell_time       (double, seconds, default 5.0)
*/
class WaypointController : public rclcpp::Node
{
public:
    WaypointController() : Node("waypoint_controller") {

        this->declare_parameter("loop_rate",           10.0);
        this->declare_parameter("waypoint_timeout",   600.0);
        this->declare_parameter("skip_on_timeout",     true);
        this->declare_parameter("default_speed",           1.0);
        this->declare_parameter("default_capture_radius", 5.0);
        this->declare_parameter("default_slip_radius",   8.0);
        this->declare_parameter("slip_dwell_time",         5.0);

        rclcpp::QoS origin_qos(1);
        origin_qos.reliable();
        origin_qos.transient_local();
        origin_sub_ = this->create_subscription<geographic_msgs::msg::GeoPoint>(
            "/origin", origin_qos,
            std::bind(&WaypointController::origin_callback, this, _1));

        waypoint_sub_ = this->create_subscription<geographic_msgs::msg::WayPoint>(
            "waypoint", 10,
            std::bind(&WaypointController::waypoint_callback, this, _1));

        this->system_control_sub_ = this->create_subscription<cougars_interfaces::msg::SystemControl>(
            "system/control", 10, std::bind(&WaypointController::system_control_callback, this, std::placeholders::_1));

        state_estimate_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/global", 10,
            std::bind(&WaypointController::state_estimate_callback, this, _1));

        command_pub_ = this->create_publisher<cougars_interfaces::msg::VehicleSetpoint>(
            "guidance/setpoint_raw", 10);

        feedback_pub_ = this->create_publisher<cougars_interfaces::msg::WaypointFeedback>(
            "waypoint_feedback", 10);

        get_parameters();
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate_)),
            std::bind(&WaypointController::control_loop, this));

        waypoint_start_time_ = this->now();
    }

private:
    void get_parameters() {
        loop_rate_ = this->get_parameter("loop_rate").as_double();
        waypoint_timeout_ = this->get_parameter("waypoint_timeout").as_double();
        skip_on_timeout_ = this->get_parameter("skip_on_timeout").as_bool();
        default_speed_ = this->get_parameter("default_speed").as_double();
        default_capture_radius_ = this->get_parameter("default_capture_radius").as_double();
        default_slip_radius_ = this->get_parameter("default_slip_radius").as_double();
        slip_dwell_time_ = this->get_parameter("slip_dwell_time").as_double();
    }

    void origin_callback(const geographic_msgs::msg::GeoPoint &msg) {
        origin_ = std::make_shared<geographic_msgs::msg::GeoPoint>(msg);
        // If a waypoint arrived before the origin was ready, convert it now
        if (current_waypoint_ && !waypoint_converted_) {
            convert_waypoint();
        }
    }

    void waypoint_callback(const geographic_msgs::msg::WayPoint::SharedPtr msg) {
        current_waypoint_ = msg;
        waypoint_converted_ = false;
        waypoint_state_ = cougars_interfaces::msg::WaypointFeedback::STATE_TRANSITING;
        waypoint_start_time_ = this->now();
        in_slip_ = false;

        if (origin_) {
            convert_waypoint();
        } else {
            RCLCPP_WARN(this->get_logger(), "Received waypoint but origin not yet available.");
        }
    }

    void system_control_callback(const cougars_interfaces::msg::SystemControl::SharedPtr msg) {
        if (msg->start.data == false) {
            RCLCPP_INFO(this->get_logger(), "Turning off WaypointController.");
            waypoint_state_ = cougars_interfaces::msg::WaypointFeedback::STATE_PARKING;
        } else {
            get_parameters();
        }
    }

    void convert_waypoint() {
        haversine_to_enu(
            origin_->latitude, origin_->longitude,
            current_waypoint_->position.latitude, current_waypoint_->position.longitude,
            waypoint_enu_x_, waypoint_enu_y_);

        waypoint_target_depth_ = current_waypoint_->position.altitude;
        waypoint_converted_ = true;
        RCLCPP_INFO(this->get_logger(), "New waypoint: ENU (%.2f, %.2f), depth %.2f m",
                    waypoint_enu_x_, waypoint_enu_y_, waypoint_target_depth_);
    }

    void state_estimate_callback(const nav_msgs::msg::Odometry &msg) {
        current_x_ = msg.pose.pose.position.x;
        current_y_ = msg.pose.pose.position.y;
        tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_heading_rad_ = yaw;
    }

    void control_loop() {
        if (!waypoint_converted_) {
            return;
        }

        if (waypoint_state_ == cougars_interfaces::msg::WaypointFeedback::STATE_ARRIVED ||
            waypoint_state_ == cougars_interfaces::msg::WaypointFeedback::STATE_SKIPPED ||
            waypoint_state_ == cougars_interfaces::msg::WaypointFeedback::STATE_PARKING) {
            return;
        }

        auto props           = parse_props(current_waypoint_->props);
        double speed         = get_prop_double(props, "speed", default_speed_);
        double cap_radius    = get_prop_double(props, "cap",   default_capture_radius_);
        double slip_radius   = get_prop_double(props, "slip",  default_slip_radius_);
        double slip_dwell_time = slip_dwell_time_;
        bool depth_from_bottom = (props.count("DFB") && props.at("DFB") == "true");
            
        double dx = waypoint_enu_x_ - current_x_;
        double dy = waypoint_enu_y_ - current_y_;
        double distance           = std::sqrt(dx * dx + dy * dy);
        double target_heading_rad = std::atan2(dy, dx);

        double heading_error_rad = target_heading_rad - current_heading_rad_;
        while (heading_error_rad >  M_PI) heading_error_rad -= 2.0 * M_PI;
        while (heading_error_rad < -M_PI) heading_error_rad += 2.0 * M_PI;

        double elapsed = (this->now() - waypoint_start_time_).seconds();
        if (elapsed > waypoint_timeout_) {
            if (skip_on_timeout_) {
                RCLCPP_WARN(this->get_logger(),
                    "Waypoint timeout after %.1f s (%.1f m away). Skipping.", elapsed, distance);
                waypoint_state_ = cougars_interfaces::msg::WaypointFeedback::STATE_SKIPPED;
                publish_feedback(distance, heading_error_rad * 180.0 / M_PI);
            }
            return;
        }

        // Capture radius: immediate arrival when very close
        if (distance < cap_radius) {
            RCLCPP_INFO(this->get_logger(), "Captured waypoint (%.2f m from target).", distance);
            waypoint_state_ = cougars_interfaces::msg::WaypointFeedback::STATE_ARRIVED;
            publish_feedback(distance, heading_error_rad * 180.0 / M_PI);
            return;
        }

        // Slip radius: arrival when the robot enters and then exits, having dwelled
        // inside for at least slip_dwell_time (filters transient GPS noise)
        if (distance < slip_radius) {
            if (!in_slip_) {
                in_slip_ = true;
                slip_entry_time_ = this->now();
            }
        } else {
            if (in_slip_) {
                double dwell = (this->now() - slip_entry_time_).seconds();
                if (dwell >= slip_dwell_time) {
                    RCLCPP_INFO(this->get_logger(),
                        "Passed through slip radius (%.2f m out, %.1f s dwell). Arrived.", distance, dwell);
                    waypoint_state_ = cougars_interfaces::msg::WaypointFeedback::STATE_ARRIVED;
                    publish_feedback(distance, heading_error_rad * 180.0 / M_PI);
                    return;
                }
                // Dwell too short — likely GPS noise; reset and keep transiting
                in_slip_ = false;
            }
        }

        auto cmd = cougars_interfaces::msg::VehicleSetpoint();
        cmd.header.stamp  = this->now();
        cmd.mode          = depth_from_bottom
                              ? cougars_interfaces::msg::VehicleSetpoint::ALTITUDE
                              : cougars_interfaces::msg::VehicleSetpoint::DEPTH;
        cmd.heading       = target_heading_rad;  // radians, ENU (0 = East)
        cmd.depth         = waypoint_target_depth_;
        cmd.speed         = speed;
        cmd.heading_valid = true;
        cmd.depth_valid   = true;
        cmd.speed_valid   = true;
        command_pub_->publish(cmd);

        publish_feedback(distance, heading_error_rad * 180.0 / M_PI);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "To waypoint: %.1f m | heading %.1f -> %.1f deg (err: %.1f deg)",
            distance,
            current_heading_rad_ * 180.0 / M_PI,
            target_heading_rad   * 180.0 / M_PI,
            heading_error_rad    * 180.0 / M_PI);
    }

    void publish_feedback(double distance, double bearing_error_deg) {
        auto fb = cougars_interfaces::msg::WaypointFeedback();
        fb.header.stamp              = this->now();
        fb.waypoint_id               = current_waypoint_->id;
        fb.state                     = waypoint_state_;
        fb.horizontal_distance_error = distance;
        fb.depth_error               = 0.0;
        fb.bearing_error             = bearing_error_deg;
        feedback_pub_->publish(fb);
    }

    // Matches the Haversine implementation in gps_odom.py
    void haversine_to_enu(double ref_lat, double ref_lon,
                          double lat,     double lon,
                          double &x,      double &y)
    {
        double ref_lat_rad = ref_lat * M_PI / 180.0;
        double ref_lon_rad = ref_lon * M_PI / 180.0;
        double lat_rad     = lat     * M_PI / 180.0;
        double lon_rad     = lon     * M_PI / 180.0;

        double delta_lon = lon_rad - ref_lon_rad;
        double delta_lat = lat_rad - ref_lat_rad;
        double a = std::sin(delta_lat / 2.0) * std::sin(delta_lat / 2.0) +
                   std::cos(ref_lat_rad) * std::cos(lat_rad) *
                   std::sin(delta_lon / 2.0) * std::sin(delta_lon / 2.0);
        double c     = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
        double d     = EARTH_RADIUS_METERS * c;
        double theta = std::atan2(
            std::sin(delta_lon) * std::cos(lat_rad),
            std::cos(ref_lat_rad) * std::sin(lat_rad) -
            std::sin(ref_lat_rad) * std::cos(lat_rad) * std::cos(delta_lon));
        x = d * std::sin(theta);
        y = d * std::cos(theta);
    }

    std::unordered_map<std::string, std::string> parse_props(
        const std::vector<geographic_msgs::msg::KeyValue> &props)
    {
        std::unordered_map<std::string, std::string> map;
        for (const auto &kv : props) { map[kv.key] = kv.value; }
        return map;
    }

    double get_prop_double(const std::unordered_map<std::string, std::string> &props,
                           const std::string &key, double default_val)
    {
        auto it = props.find(key);
        if (it != props.end()) {
            try { return std::stod(it->second); } catch (...) {}
        }
        return default_val;
    }

    rclcpp::Subscription<geographic_msgs::msg::GeoPoint>::SharedPtr                    origin_sub_;
    rclcpp::Subscription<geographic_msgs::msg::WayPoint>::SharedPtr                    waypoint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                           state_estimate_sub_;
    rclcpp::Subscription<cougars_interfaces::msg::SystemControl>::SharedPtr            system_control_sub_;

    rclcpp::Publisher<cougars_interfaces::msg::VehicleSetpoint>::SharedPtr   command_pub_;
    rclcpp::Publisher<cougars_interfaces::msg::WaypointFeedback>::SharedPtr  feedback_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    std::shared_ptr<geographic_msgs::msg::GeoPoint>  origin_;
    geographic_msgs::msg::WayPoint::SharedPtr        current_waypoint_;
    bool   waypoint_converted_    = false;
    double waypoint_enu_x_        = 0.0;
    double waypoint_enu_y_        = 0.0;
    double waypoint_target_depth_ = 0.0;

    double current_x_           = 0.0;
    double current_y_           = 0.0;
    double current_heading_rad_ = 0.0;

    uint8_t        waypoint_state_      = cougars_interfaces::msg::WaypointFeedback::STATE_TRANSITING;
    rclcpp::Time   waypoint_start_time_;
    bool           in_slip_             = false;
    rclcpp::Time   slip_entry_time_;

    // Parameters
    double loop_rate_;
    double waypoint_timeout_;
    bool   skip_on_timeout_;
    double default_speed_;
    double default_capture_radius_;
    double default_slip_radius_;
    double slip_dwell_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointController>());
    rclcpp::shutdown();
    return 0;
}
