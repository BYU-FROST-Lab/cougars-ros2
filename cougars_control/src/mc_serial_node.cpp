#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include "cougars_interfaces/msg/u_command.hpp"
#include <libserialport.h>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <sstream>
// Added these libraries for the real path resolution
#include <stdlib.h>
#include <limits.h>
//todo: recognize which microcontroller is being used, possibly through bash variable $UCONTROLLER???
//ucontroller env var sourced in launch file
class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("mc_serial_node") {
        // Initialize subscriber
        // TODO: Look at remappings for holoocean namespace??
        control_command_sub_ = this->create_subscription<cougars_interfaces::msg::UCommand>(
            "kinematics/command", 10, std::bind(&ControlNode::controlCommandCallback, this, std::placeholders::_1));



        battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery/data", 10);

        leak_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("leak/data", 10);
        


        
        this->declare_parameter("demo_mode", false);
        this->declare_parameter("UCONTROLLER", "TEENSY");
        this->declare_parameter("UCONTROLLER_SERIAL", "/dev/frost/teensy");
        sp_set_baudrate(serial_port_, 115200);
        // Open the serial port


        //Added this code so that we can use udev symlink path
        //This function finds the real path within the udev rule so that it can use the real path with the libserialport library
        //libserial port library cannot use udev paths
        char resolved_path[PATH_MAX];
        if (realpath(this->get_parameter("UCONTROLLER_SERIAL").as_string().c_str(), resolved_path) != NULL) {
                std::cout << "Real path: " << resolved_path << std::endl;
        } else {
            std::cerr << "Error resolving path" << std::endl;
        }
        
        if (sp_get_port_by_name(resolved_path, &serial_port_) != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Unable to find port");
            rclcpp::shutdown();
        }

        if (sp_open(serial_port_, SP_MODE_READ_WRITE) != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port");
            sp_free_port(serial_port_);
            rclcpp::shutdown();
        }

        if(this->get_parameter("UCONTROLLER").as_string() == "STM"){
            //config UART
            sp_set_baudrate(serial_port_, 115200);
            sp_set_bits(serial_port_, 8);
            sp_set_parity(serial_port_, SP_PARITY_EVEN);
            sp_set_stopbits(serial_port_, 1);
            sp_set_flowcontrol(serial_port_, SP_FLOWCONTROL_NONE);

            dvl_sw_service_ = this->create_service<std_srvs::srv::SetBool>(
            "dvl_modem_enable", std::bind(&ControlNode::dvlServiceCallback, this, std::placeholders::_1,std::placeholders::_2));

        } else{
            // Initialize pressure publisher
            pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("pressure/data", 10);
            
        }



        RCLCPP_INFO(this->get_logger(), "Serial port initialized");

        // Timer to read from serial port
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ControlNode::readSerialData, this));
    }

    ~ControlNode() {
        sp_close(serial_port_);
        sp_free_port(serial_port_);
    }

private:
    void controlCommandCallback(const cougars_interfaces::msg::UCommand::SharedPtr msg) {
        std::stringstream ss;
        // (Currently have 3 fins)
        ss << "$CONTR," << msg->fin[0] << "," << msg->fin[1] << "," << msg->fin[2] << "," << msg->thruster << "\n";
        std::string command = ss.str();
        sp_nonblocking_write(serial_port_, command.c_str(), command.size());
        // RCLCPP_INFO(this->get_logger(), "Sent command: %s", command.c_str());
        std::cout << "Sent command: " << command.c_str() << std::endl;
    }
    void dvlServiceCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
        response->success = true;
        std::stringstream ss;
        ss << "$CONTR2," << request->data << ",5\n"; //no output to the led blink, can be implemented in the future
        std::string command = ss.str();
        sp_nonblocking_write(serial_port_, command.c_str(), command.size());
        response->message = "message sent";

    }
    void readSerialData() {
        char temp_buffer[1024];
        int bytes_read = sp_nonblocking_read(serial_port_, temp_buffer, sizeof(temp_buffer) - 1);
        
        if (bytes_read > 0) {
            temp_buffer[bytes_read] = '\0';
            buffer_ += temp_buffer;

            std::vector<std::string> messages = splitMessages(buffer_);

            // std::cout << "Split messages:\n";
            for (const std::string& msg : messages) {
                // std::cout << "message: [" << msg << "]" << std::endl;
                processMessage(msg);  // process each message individually
            }
        } else if (bytes_read < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error reading from serial port");
        }
    }

    std::vector<std::string> splitMessages(std::string& data) {
        std::vector<std::string> messages;

        size_t start = 0;
        size_t end = 0;

        while ((end = data.find('\n', start)) != std::string::npos) {
            std::string msg = data.substr(start, end - start);

            // Trim carriage return if present
            if (!msg.empty() && msg.back() == '\r') {
                msg.pop_back();
            }

            if (!msg.empty()) {
                messages.push_back(msg);
            }

            start = end + 1;
        }

        // Remainder goes back to buffer
        data = data.substr(start);

        return messages;
    }

    void processMessage(const std::string& message) {
        if (message.rfind("$DEPTH", 0) == 0) {
            double pressure, temperature;
            if (sscanf(message.c_str(), "$DEPTH,%lf,%lf", &pressure, &temperature) == 2) {
                sensor_msgs::msg::FluidPressure pressure_msg;
                pressure_msg.header.stamp = this->now();
                pressure_msg.fluid_pressure = pressure;
                pressure_msg.variance = 0.0;  // Variance can be set if known

                if (!this->get_parameter("demo_mode").as_bool()){
                    pressure_pub_->publish(pressure_msg);
                    std::cout << "Published pressure: " << pressure << std::endl;
                }
                else {
                    // In demo mode, we can log the pressure without publishing
                    // std::cout << "Demo mode: Pressure not published, value: " << pressure << std::endl;
                    RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(), 5000, "Demo mode: Pressure not published, value: %f", pressure);
                }
                
            }
        }
        if (message.rfind("$BATTE", 0) == 0){
            double voltage, current;
            if(sscanf(message.c_str(), "$BATTE,%lf,%lf", &voltage, &current) == 2){
                sensor_msgs::msg::BatteryState battery_msg;
                battery_msg.voltage = voltage;
                battery_msg.current = current;
                battery_msg.header.stamp = this->now();

                battery_pub_->publish(battery_msg);
            }
        }
        if (message.rfind("$LEAK", 0) == 0) {
            int leak;
            if (sscanf(message.c_str(), "$LEAK,%d", &leak) == 1) {
                sensor_msgs::msg::FluidPressure leak_msg;
                leak_msg.header.stamp = this->now();
                leak_msg.fluid_pressure = leak;
                leak_msg.header.stamp = this->now();
                leak_pub_->publish(leak_msg);
            }
        }
        // Add processing for other message types (BATTE, LEAK) here if needed
    }


    std::string buffer_;
    const char delimiter_ = '\n';

    rclcpp::Subscription<cougars_interfaces::msg::UCommand>::SharedPtr control_command_sub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr leak_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr dvl_sw_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct sp_port *serial_port_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
