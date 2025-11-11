#include "rclcpp/rclcpp.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"
#include <seatrac_driver/SeatracEnums.h>


#include "cougars_coms/coms_protocol.hpp"


#include <iostream>
#include <chrono>
#include <memory>

using namespace std::literals::chrono_literals;
using namespace cougars_coms;
using namespace narval::seatrac;

using std::placeholders::_1;
using std::placeholders::_2;

class VehiclePinger : public rclcpp::Node {
public:
    VehiclePinger() : Node("vehicle_pinger") {

        // list of beacon IDs to ping
        this->declare_parameter<std::vector<int64_t>>("vehicles_in_mission", {1, 2});
        this->vehicles_in_mission_ = this->get_parameter("vehicles_in_mission").as_integer_array();
        
        // frequency of pings in seconds
        this->declare_parameter<int>("ping_frequency_seconds", 4);
        this->ping_frequency = this->get_parameter("ping_frequency_seconds").as_int();

        // whether to enable pinging
        this->declare_parameter<bool>("enable_ping", false);
        bool enable_ping = this->get_parameter("enable_ping").as_bool();

        this->declare_parameter<bool>("use_ping", true);
        this->use_ping = this->get_parameter("use_ping").as_bool();

        if (enable_ping) {
            this->modem_publisher_ = this->create_publisher<seatrac_interfaces::msg::ModemSend>("modem_send", 10);
            timer_ = this->create_wall_timer(std::chrono::seconds(ping_frequency), std::bind(&VehiclePinger::ping_schedule, this));
        }

    }
    
    // iterates through the list of vehicles and sends a ping to next vehicle every time called
    void ping_schedule() {
        vehicle_id_index += 1;
        if (vehicle_id_index >= vehicles_in_mission_.size())
            vehicle_id_index = 0;

        this->send_ping(vehicles_in_mission_[vehicle_id_index]);
    }

    // sends a ping to the specified vehicle ID requesting localization info
    void send_ping(int target_id) {

        RCLCPP_INFO(this->get_logger(), "Pinging vehicle with ID: %d", target_id);
        auto request = seatrac_interfaces::msg::ModemSend();
        request.dest_id = (uint8_t)target_id;
        request.msg_type = MSG_REQX;
        if (this->use_ping) {
            request.msg_id = CID_PING_SEND; // Just a ping message
        } else {
            request.msg_id = CID_DAT_SEND; // Gets localization info
            RequestLocalizationInfo message;
            request.packet_len = (uint8_t)std::min((int)sizeof(message), 31);
            std::memcpy(&request.packet_data, &message, request.packet_len);
        }
        this->modem_publisher_->publish(request);
    }

private:

    rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_publisher_;

    std::vector<int64_t> vehicles_in_mission_;
    size_t vehicle_id_index = -1;  // Index to track which vehicle to ping next
    int ping_frequency;  // Frequency of pings in seconds
    rclcpp::TimerBase::SharedPtr timer_;

    bool use_ping;  // Whether to use pinging functionality
    
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto vehicle_pinger_node = std::make_shared<VehiclePinger>();
  rclcpp::spin(vehicle_pinger_node);
  rclcpp::shutdown();
  return 0;
}
