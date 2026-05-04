#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>

// DVL messages
#include <dvl_msgs/msg/dvl.hpp>
#include <dvl_msgs/msg/dvldr.hpp>

// Sensor Messages
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// Seatrac Interfaces
#include <seatrac_interfaces/msg/modem_rec.hpp>
#include <seatrac_interfaces/msg/modem_status.hpp>
#include <seatrac_interfaces/msg/modem_send.hpp>
#include <seatrac_interfaces/msg/modem_cmd_update.hpp>

// Frost interfaces
#include <cougars_interfaces/msg/u_command.hpp>
#include <cougars_interfaces/msg/controls_debug.hpp>
#include <cougars_interfaces/msg/control_command.hpp>
#include <cougars_interfaces/msg/system_control.hpp>
#include <cougars_interfaces/msg/system_status.hpp>

// Geometry Messages
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

// Other
#include <std_msgs/msg/int32.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>


using std::placeholders::_1;

class MultiTopicBagRecorder : public rclcpp::Node
{
public:
    MultiTopicBagRecorder() : Node("multi_topic_bag_recorder")
    {
        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        write_flag_ = false;


        system_control_sub_ = this->create_subscription<cougars_interfaces::msg::SystemControl>(
            "system/status", 1, std::bind(&MultiTopicBagRecorder::system_callback, this, _1));

        const std::string home = getenv("HOME") ? std::string(getenv("HOME")) : std::string("/home/frostlab");

        this->declare_parameter<bool>("sensors", true);
        this->declare_parameter<bool>("system", true);
        this->declare_parameter<bool>("processed", true);
        this->declare_parameter<bool>("controls", true);
        this->declare_parameter<std::string>("bag_dir", home + "/bag/");
        this->declare_parameter<std::string>("mission_file_path", "");
        this->declare_parameter<std::string>("vehicle_params_file", home + "/config/agent/vehicle_params.yaml");
        this->declare_parameter<std::string>("fleet_params_file", home + "/config/fleet/fleet_params.yaml");

        bool sensors, system, processed, controls;
        this->get_parameter("sensors", sensors);
        this->get_parameter("system", system);
        this->get_parameter("processed", processed);
        this->get_parameter("controls", controls);

        if (sensors) {
            // Sensor Data
            subscribe_to_topic<dvl_msgs::msg::DVL>("dvl/data");
            subscribe_to_topic<dvl_msgs::msg::DVLDR>("dvl/position");
            subscribe_to_topic<gps_msgs::msg::GPSFix>("extended_fix");
            subscribe_to_topic<sensor_msgs::msg::NavSatFix>("fix");
            subscribe_to_topic<sensor_msgs::msg::FluidPressure>("pressure/data");
            subscribe_to_topic<sensor_msgs::msg::BatteryState>("battery/data");
            subscribe_to_topic<sensor_msgs::msg::FluidPressure>("leak/data");
            subscribe_to_topic<seatrac_interfaces::msg::ModemRec>("modem_rec");
            subscribe_to_topic<seatrac_interfaces::msg::ModemSend>("modem_send");
            subscribe_to_topic<seatrac_interfaces::msg::ModemCmdUpdate>("modem_cmd_update");
            subscribe_to_topic<seatrac_interfaces::msg::ModemStatus>("modem_status");
        }

        if (system){
            // System
            subscribe_to_topic<cougars_interfaces::msg::SystemStatus>("safety_status");
            subscribe_to_topic<diagnostic_msgs::msg::DiagnosticArray>("diagnostics");
        }


        if (processed){
            // Processed Data
            subscribe_to_topic<geometry_msgs::msg::PoseWithCovarianceStamped>("dvl/dead_reckoning");
            subscribe_to_topic<geometry_msgs::msg::PoseWithCovarianceStamped>("dvl/dr_global");
            subscribe_to_topic<geometry_msgs::msg::TwistWithCovarianceStamped>("dvl/velocity");
            subscribe_to_topic<nav_msgs::msg::Odometry>("gps/odom");
            subscribe_to_topic<nav_msgs::msg::Odometry>("smoothed_output");
            subscribe_to_topic<sensor_msgs::msg::Imu>("modem_imu");
            subscribe_to_topic<geometry_msgs::msg::PoseWithCovarianceStamped>("depth_data");

        }
        
        if (controls){
            // Mission and Controls
            subscribe_to_topic<cougars_interfaces::msg::UCommand>("kinematics/command");
            subscribe_to_topic<cougars_interfaces::msg::UCommand>("controls/command");
            subscribe_to_topic<cougars_interfaces::msg::ControlsDebug>("controls/debug");
            subscribe_to_topic<cougars_interfaces::msg::ControlCommand>("control_command");
        }

    }

private:
    template <typename MsgT>
    void subscribe_to_topic(const std::string &topic_name)
    {
        auto callback = [this, topic_name](typename MsgT::SharedPtr msg)
        {
            if(write_flag_){
                rclcpp::Serialization<MsgT> serializer;
                rclcpp::SerializedMessage serialized_msg;
                serializer.serialize_message(msg.get(), &serialized_msg);

                rclcpp::Time time_stamp = this->now();
                writer_->write(
                    std::make_shared<rclcpp::SerializedMessage>(serialized_msg),
                    topic_name,
                    rosidl_generator_traits::name<MsgT>(),
                    time_stamp
                );
            }
        };

        auto qos = rclcpp::SensorDataQoS();

        auto sub = create_subscription<MsgT>(topic_name, qos, callback);
        subscriptions_.push_back(sub);
    }

    void system_callback(const cougars_interfaces::msg::SystemControl::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received system control message: start=%d, rosbag_flag=%d", msg->start.data, msg->rosbag_flag.data);

        if (msg->start.data && msg->rosbag_flag.data) {
            std::string bag_folder = get_bag_filename(msg->rosbag_prefix);

            writer_->open(bag_folder);
            write_flag_ = true;

            std::string mission_file, vehicle_params, fleet_params;
            this->get_parameter("mission_file_path", mission_file);
            this->get_parameter("vehicle_params_file", vehicle_params);
            this->get_parameter("fleet_params_file", fleet_params);

            copy_file_to_bag(mission_file, bag_folder, "mission file");
            copy_file_to_bag(vehicle_params, bag_folder, "vehicle params");
            copy_file_to_bag(fleet_params, bag_folder, "fleet params");

        } else {
            write_flag_ = false;
            writer_->close();
        }
    }

    void copy_file_to_bag(const std::string& src, const std::string& bag_folder, const std::string& label)
    {
        if (src.empty()) {
            RCLCPP_WARN(this->get_logger(), "%s path is not set or empty", label.c_str());
            return;
        }
        try {
            std::filesystem::path dst = std::filesystem::path(bag_folder) / std::filesystem::path(src).filename();
            std::filesystem::copy_file(src, dst, std::filesystem::copy_options::overwrite_existing);
            RCLCPP_INFO(this->get_logger(), "Copied %s to: %s", label.c_str(), dst.c_str());
        } catch (std::filesystem::filesystem_error& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to copy %s: %s", label.c_str(), e.what());
        }
    }

    std::string get_bag_filename(const std::string& prefix = "rosbag")
    {
        std::string bag_dir;
        this->get_parameter("bag_dir", bag_dir);
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << bag_dir << prefix << "_" << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
        return ss.str();
    }

    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    rclcpp::Subscription<cougars_interfaces::msg::SystemControl>::SharedPtr system_control_sub_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;

    bool write_flag_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiTopicBagRecorder>());
    rclcpp::shutdown();
    return 0;
}
