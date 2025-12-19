



#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "dvl_msgs/msg/dvldr.hpp"
#include "dvl_msgs/msg/dvl.hpp"
#include "cougars_interfaces/msg/system_status.hpp"
#include "cougars_interfaces/msg/localization_data.hpp"
#include "cougars_interfaces/msg/localization_data_short.hpp"
#include "cougars_interfaces/msg/system_control.hpp"
#include "cougars_interfaces/msg/way_point.hpp"

#include <seatrac_driver/SeatracEnums.h>




#include "cougars_coms/coms_protocol.hpp"


#include <iostream>
#include <chrono>
#include <memory>


using namespace std::literals::chrono_literals;
using std::placeholders::_1;
using namespace cougars_coms;
using namespace narval::seatrac;


class ComsNode : public rclcpp::Node {
public:
    ComsNode() : Node("cougars_coms") {

        // id of base station beacon, used for messages sent to the base station
        this->base_station_beacon_id_ = this->declare_parameter<int>("base_station_beacon_id", 15);

        //buffer between immediate response to localization requests and secondary packet response
        this->localization_queue_buffer = this->declare_parameter<int>("localization_queue_buffer_ms", 1000);

        this->collision_guard_wait_ = this->declare_parameter<double>("collision_guard_wait_sec", 3.0);


        // subscriber for safety status messages
        this->safety_subscriber_ = this->create_subscription<cougars_interfaces::msg::SystemStatus>(
            "safety_status", 10,
            std::bind(&ComsNode::safety_callback, this, _1)
        );

        // subscriber for battery data
        this->battery_subscriber_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery/data", 10,
            std::bind(&ComsNode::battery_callback, this, _1)
        );

        // subscriber for pressure data
        this->pressure_subscriber_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "pressure/data", 10,
            std::bind(&ComsNode::pressure_callback, this, _1)
        );

        // subscriber for depth data
        this->depth_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "depth_data", 10,

            [this](geometry_msgs::msg::PoseWithCovarianceStamped msg) {
                this->depth = msg.pose.pose.position.z;
            }
        );

        this->waypoint_subscriber_ = this->create_subscription<cougars_interfaces::msg::WayPoint>(
            "current_waypoint", 10,
            [this](cougars_interfaces::msg::WayPoint msg) {
                this->waypoint_num = static_cast<uint8_t>(msg.waypoint_num);
                this->waypoint_x = static_cast<uint16_t>(msg.x);
                this->waypoint_y = static_cast<uint16_t>(msg.y);
                this->waypoint_depth = static_cast<uint8_t>(msg.depth); 
            }
        );

        //subscriber for dvl data
        this->dvl_subscriber_ = this->create_subscription<dvl_msgs::msg::DVLDR>(
            "dvl/position", 10,
            [this](dvl_msgs::msg::DVLDR msg) {
                this->dvl_position_x = msg.position.x;
                this->dvl_position_y = msg.position.y;
                this->dvl_position_z = msg.position.z;
                this->roll = msg.roll;
                this->pitch = msg.pitch;
                this->yaw = msg.yaw;
            }
        );

        this->dvl_vel_subscriber_ = this->create_subscription<dvl_msgs::msg::DVL>(
            "dvl/data", 10,
            [this](dvl_msgs::msg::DVL msg) {
                this->velocity_x = msg.velocity.x;
                this->velocity_y = msg.velocity.y;
                this->velocity_z = msg.velocity.z;
            }
        );

        // subscriber to modem rec messages, sent when the modem receives a message
        this->modem_subscriber_ = this->create_subscription<seatrac_interfaces::msg::ModemRec>(
            "modem_rec", 10,
            std::bind(&ComsNode::listen_to_modem, this, _1)
        );

        // publisher for sending messages via the modem
        this->modem_publisher_ = this->create_publisher<seatrac_interfaces::msg::ModemSend>("modem_send", 10);

        // client for thruster control service
        this->thruster_client_ = this->create_client<std_srvs::srv::SetBool>(
            "arm_thruster"
        );

        // client for surface control service
        this->surface_client_ = this->create_client<std_srvs::srv::SetBool>(
            "surface"
        );

        // publisher to start mission
        this->init_publisher_ = this->create_publisher<cougars_interfaces::msg::SystemControl>(
            "system/status", 10
        );

        // publisher for full localization data
        this->localization_data_publisher_ = this->create_publisher<cougars_interfaces::msg::LocalizationData>("localization_data", 10);

        // publisher for short localization data
        this->localization_data_short_publisher_ = this->create_publisher<cougars_interfaces::msg::LocalizationDataShort>("localization_data_short", 10);

        // timer for queuing localization responses
        this->localization_queue_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(this->localization_queue_buffer),
            std::bind(&ComsNode::queue_localization_response, this)
        );

        // initialize the last modem message time to 10 seconds ago
        last_modem_msg_time_ = this->now() - rclcpp::Duration::from_seconds(10); // 10s ago

    }

    // called when a message is revieved through the modem
    // this function checks the message id and calls the appropriate function
    void listen_to_modem(seatrac_interfaces::msg::ModemRec msg) {
        this->last_modem_msg_time_ = this->now(); // Update the last modem message time
        COUG_MSG_ID id = (COUG_MSG_ID)msg.packet_data[0];
        switch(id) {
            default: break;
            case EMPTY: {
                record_range_and_azimuth(msg);
            } break;
            case REQUEST_STATUS: {
                send_status();
            } break;
            case INIT: {
                init(&msg);
            } break;
            case EMERGENCY_KILL: {
                kill_thruster();
            } break;
            case EMERGENCY_SURFACE: {
                emergency_surface();
            } break;
            case LOCALIZATION_INFO: {
               record_localization_info(msg);
            } break;

        }
    }

    // updates battery data
    void battery_callback(sensor_msgs::msg::BatteryState msg) {
        // Here you can handle the battery data if needed
        this->battery_voltage = msg.voltage; 
    }

    // makes a bit mask that has safety status message
    void safety_callback(cougars_interfaces::msg::SystemStatus msg) {
        // Here you can handle the safety system status data if needed
        this->safety_mask =
            (msg.depth_status.data      ? 1 << 0 : 0) |
            (msg.gps_status.data        ? 1 << 1 : 0) |
            (msg.modem_status.data      ? 1 << 2 : 0) |
            (msg.dvl_status.data        ? 1 << 3 : 0) |
            (msg.emergency_status.data  ? 1 << 4 : 0);
    }

    // updates pressure data
    void pressure_callback(sensor_msgs::msg::FluidPressure msg) {
        this->pressure = msg.fluid_pressure; 
    }

    void init(seatrac_interfaces::msg::ModemRec* msg) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing system.");
        cougars_interfaces::msg::SystemControl control_msg;
        const Init* msg_recieved = reinterpret_cast<const Init*>(msg->packet_data.data());
        control_msg.start.data = (msg_recieved->init_bitmask & 0x01) != 0;
        control_msg.rosbag_flag.data = (msg_recieved->init_bitmask & 0x02) != 0;
        control_msg.thruster_arm.data = (msg_recieved->init_bitmask & 0x04) != 0;
        control_msg.dvl_acoustics.data = (msg_recieved->init_bitmask & 0x08) != 0;
        // turns char[32] to std::string
        control_msg.rosbag_prefix = std::string(msg_recieved->rosbag_prefix);
        this->init_publisher_->publish(control_msg);
    }

    // kills the thruster using the arm_thruster service
    void kill_thruster() {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received kill command from base station");
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = false;
        while (!this->thruster_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                    "Interrupted while waiting for the arm_thruster service. Exiting.");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "arm_thruster service not available, waiting again...");
        }


        auto result_future = this->thruster_client_->async_send_request(request,
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response_future) {
                try {
                    auto response = response_future.get();
                    if (response->success) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Thruster has been deactivated.");
                    } else {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to deactivate thruster.");
                    }
                    ConfirmEmergencyKill msg;
                    msg.success = response->success;
                    this->send_acoustic_message(base_station_beacon_id_, 1, (uint8_t*)&msg);
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Error while trying to deactivate thruster: %s", e.what());
                }
            }
        );
    }

    // sends a surface command to the vehicle using the surface service
    void emergency_surface() {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received surface command from base station");
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        while (!this->surface_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                    "Interrupted while waiting for the surface service. Exiting.");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "surface service not available, waiting again...");
        }


        auto result_future = this->surface_client_->async_send_request(request,
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response_future) {
                try {
                    auto response = response_future.get();
                    if (response->success) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Surface command was successfull.");
                    } else {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Surface command failed.");
                    }
                    ConfirmEmergencySurface msg;
                    msg.success = response->success;
                    this->send_acoustic_message(base_station_beacon_id_, 1, (uint8_t*)&msg);
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Error while trying to send surface command: %s", e.what());
                }
            }
        );
    }

    // sends a status message to the base station
    void send_status(){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending status to base station.");
        VehicleStatus status_msg;

        //fill in x and y depending on whether we are using factor graph or DVL
        status_msg.waypoint_num = 0; // Placeholder, set to 0 for now
        status_msg.waypoint_x = this->position_x;
        status_msg.waypoint_y = this->position_y;
        status_msg.waypoint_depth = this->position_z;
        
        status_msg.x = this->dvl_position_x;
        status_msg.y = this->dvl_position_y;
        status_msg.battery_voltage = this->battery_voltage;
        status_msg.safety_mask = this->safety_mask;
        status_msg.roll = this->roll;
        status_msg.pitch = this->pitch;
        status_msg.yaw = this->yaw;
        status_msg.depth = this->depth; 
        status_msg.x_vel = this->velocity_x;
        status_msg.y_vel = this->velocity_y;
        status_msg.z_vel = this->velocity_z;
        status_msg.pressure = this->pressure;
        send_acoustic_message(base_station_beacon_id_, sizeof(status_msg), (uint8_t*)&status_msg);
    }

    // records localization info from message received
    void record_localization_info(seatrac_interfaces::msg::ModemRec msg) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received localization info from vehicle %d.", msg.src_id);

        if (msg.packet_len < sizeof(LocalizationInfo)) {
            RCLCPP_ERROR(this->get_logger(),
                "packet_data too small for LocalizationInfo! Got %u bytes, expected %zu",
                msg.packet_len, sizeof(LocalizationInfo));
            return;
        }   

        const LocalizationInfo* info = reinterpret_cast<const LocalizationInfo*>(msg.packet_data.data());
        cougars_interfaces::msg::LocalizationData localization_data;
        localization_data.header.stamp = this->now();
        localization_data.vehicle_id = msg.src_id;
        localization_data.x = info->x;
        localization_data.y = info->y;
        localization_data.z = info->z;
        localization_data.roll = info->roll;
        localization_data.pitch = info->pitch;
        localization_data.yaw = info->yaw;
        localization_data.depth = info->depth;
        localization_data.range = msg.range_dist;
        localization_data.azimuth = msg.usbl_azimuth;
        localization_data.elevation = msg.usbl_elevation;


        localization_data_publisher_->publish(localization_data);

        RCLCPP_INFO(this->get_logger(), "Localization Info for vehicle %d: (x, y, z): (%.2f, %.2f, %.2f), (roll, pitch, yaw): (%.2f, %.2f, %.2f), depth: %.2f, range: %.2f, azimuth: %.2f",
            localization_data.vehicle_id, localization_data.x, localization_data.y, localization_data.z,
            localization_data.roll, localization_data.pitch, localization_data.yaw,
            localization_data.depth, localization_data.range, localization_data.azimuth);
   }

    // checks if the range and azimuth data is included in the message and publishes it if so
   void record_range_and_azimuth(seatrac_interfaces::msg::ModemRec msg) {
        if (msg.includes_range && msg.includes_usbl) {
            RCLCPP_INFO(this->get_logger(), "Vehicle %d:  Range distance: %d, Azimuth: %i, Elevation: %i, depth: %i", msg.src_id, msg.range_dist, msg.usbl_azimuth, msg.usbl_elevation, msg.position_depth);
            cougars_interfaces::msg::LocalizationDataShort localization_data_short;
            localization_data_short.header.stamp = this->now();
            localization_data_short.vehicle_id = msg.src_id;
            localization_data_short.range = msg.range_dist;
            localization_data_short.azimuth = msg.usbl_azimuth;
            localization_data_short.elevation = msg.usbl_elevation;
            localization_data_short.depth = msg.position_depth;
            this->localization_data_short_publisher_->publish(localization_data_short);
        }
    }

    void queue_localization_response() {
        if ((this->now() - this->last_modem_msg_time_).seconds() >= this->collision_guard_wait_) {
            LocalizationInfo message;
            message.x = this->dvl_position_x;
            message.y = this->dvl_position_y;
            message.z = this->dvl_position_z;
            message.roll = this->roll;
            message.pitch = this->pitch;
            message.yaw = this->yaw;
            message.depth = this->depth;

            send_acoustic_message(0, sizeof(message), (uint8_t*)&message, CID_DAT_QUEUE_SET);

            RCLCPP_DEBUG(this->get_logger(), "[%ld] Queued localization data", this->now().nanoseconds());
        }
    }

    // sends an acoustic message to a target vehicle
    void send_acoustic_message(int target_id, int message_len, uint8_t* message, CID_E msg_id = CID_DAT_SEND) {
        auto request = seatrac_interfaces::msg::ModemSend();
        request.msg_id = msg_id;
        request.dest_id = (uint8_t)target_id;
        request.msg_type = MSG_OWAY; //MSG_OWAY, data sent one way without response or position data
        request.packet_len = (uint8_t)std::min(message_len, 31);
        std::memcpy(&request.packet_data, message, request.packet_len);
       
        this->modem_publisher_->publish(request);
    }


private:


    rclcpp::Subscription<seatrac_interfaces::msg::ModemRec>::SharedPtr modem_subscriber_;
    rclcpp::Subscription<cougars_interfaces::msg::SystemStatus>::SharedPtr safety_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_subscriber_;
    rclcpp::Subscription<dvl_msgs::msg::DVLDR>::SharedPtr dvl_subscriber_;
    rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr dvl_vel_subscriber_;
    rclcpp::Subscription<cougars_interfaces::msg::WayPoint>::SharedPtr waypoint_subscriber_;

    rclcpp::Publisher<cougars_interfaces::msg::LocalizationData>::SharedPtr localization_data_publisher_;
    rclcpp::Publisher<cougars_interfaces::msg::LocalizationDataShort>::SharedPtr localization_data_short_publisher_;
    rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_publisher_;
    rclcpp::Publisher<cougars_interfaces::msg::SystemControl>::SharedPtr init_publisher_;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr thruster_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr surface_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr init_controls_client_;



    uint8_t position_x;
    uint8_t position_y;
    uint8_t position_z;
    uint8_t velocity_x;
    uint8_t velocity_y;
    uint8_t velocity_z;
    uint8_t base_station_beacon_id_;
    uint8_t safety_mask;
    uint8_t battery_voltage;
    uint8_t heading;
    uint8_t pressure;
    uint8_t waypoint_num;
    uint16_t waypoint_x;
    uint16_t waypoint_y;
    uint8_t waypoint_depth;

    float dvl_position_x;
    float dvl_position_y;
    float dvl_position_z;
    float roll;
    float pitch;
    float yaw;
    float depth;
     

    float recent_range;
    float recent_azimuth;
    float recent_elevation;

    int localization_queue_buffer;
    float collision_guard_wait_;
    rclcpp::TimerBase::SharedPtr localization_queue_timer_;
    bool use_factor_graph_;

    rclcpp::Time last_modem_msg_time_;

};




int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto coms_node = std::make_shared<ComsNode>();
  rclcpp::spin(coms_node);
  rclcpp::shutdown();
  return 0;
}



