#include <memory>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <numeric>
#include <functional>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "cougars_interfaces/msg/u_command.hpp"
#include "cougars_interfaces/msg/system_status.hpp"
#include "std_msgs/msg/int32.hpp"
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "gps_msgs/msg/gps_fix.hpp"
#include "dvl_msgs/msg/dvldr.hpp"
#include "seatrac_interfaces/msg/modem_status.hpp"
#include "geographic_msgs/msg/geo_point.hpp"

using std::placeholders::_1;

class EmergencyProtocols : public rclcpp::Node
{


    public:
    EmergencyProtocols()
    : Node("emergency_protocols")
    {

        this->declare_parameter("modem_connection_timeout", 2);
        this->declare_parameter("dvl_connection_timeout", 2);
        this->declare_parameter("gps_connection_timeout", 10);
        this->declare_parameter("gps_origin_dist_max", 0.0001);
        this->declare_parameter("verbosity", 1); // 0 for no error logging, 1 for single error per problem, 2 for repeated error logging

        this->modem_status_subscription_ = this->create_subscription<seatrac_interfaces::msg::ModemStatus>(
            "modem_status", 10,
            std::bind(&EmergencyProtocols::modem_callback, this, _1)
        );

        this->dvl_subscription_ = this->create_subscription<dvl_msgs::msg::DVLDR>(
            "dvl/position", 10,
            std::bind(&EmergencyProtocols::dvl_callback, this, _1)
        );

        this->gps_subscription_ = this->create_subscription<gps_msgs::msg::GPSFix>(
            "fix", 10,
            std::bind(&EmergencyProtocols::gps_callback, this, _1)
        );

        this->gps_origin_subscription_ = this->create_subscription<geographic_msgs::msg::GeoPoint>(
            "origin", 10,
            std::bind(&EmergencyProtocols::origin_callback, this, _1)
        );

        this->status_publisher_ = this->create_publisher<cougars_interfaces::msg::SystemStatus>("safety_status", 10);

        // wait 30 seconds after startup before monitoring, to give time for all the nodes to start and publish their first messages
        // only goes off once, and then starts the regular monitoring timers
        this->startup_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30000),
            std::bind(&EmergencyProtocols::startup_callback, this)
        );


        this->modem_conection_logged = false;
        this->dvl_conection_logged = false;
        this->dvl_connected = false;
        this->modem_conected = false;
        this->gps_conection_logged = false;
        this->gps_origin_lat = 0.0;
        this->gps_origin_lon = 0.0;
        this->gps_origin_alt = 0.0;


        get_parameters();
    }
    private:

    void get_parameters() {
        this->modem_connection_timeout_ = this->get_parameter("modem_connection_timeout").as_int();
        this->dvl_connection_timeout_ = this->get_parameter("dvl_connection_timeout").as_int();
        this->gps_connection_timeout_ = this->get_parameter("gps_connection_timeout").as_int();
        this->gps_origin_dist_max_ = this->get_parameter("gps_origin_dist_max").as_double();
        this->verbosity_ = this->get_parameter("verbosity").as_int();
    }

    void startup_callback() {
        RCLCPP_INFO(this->get_logger(), "Starting emergency protocols monitoring.");

        // timer to monitor the connections of the various systems, e.g. modem, gps, dvl, etc.
        this->monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(3000),
            std::bind(&EmergencyProtocols::monitor_sensors, this)
        );

        this->status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&EmergencyProtocols::publish_status, this)
        );

        this->startup_timer_->cancel(); // cancel the startup timer, we only want it to go off once
    }

    void monitor_sensors(){
        monitor_modem();
        monitor_gps();
        monitor_dvl();
        // monitor_radio();
        // monitor_battery();
    }

    void publish_status(){
        cougars_interfaces::msg::SystemStatus message;
        if (!this->dvl_connected){
            message.dvl_status.set__data(1);
        } else {
            message.dvl_status.set__data(0);
        }
        if (!this->modem_conected){
            message.modem_status.set__data(1);
        } else {
            message.modem_status.set__data(0);
        }
        if (!this->received_gps){
            message.gps_status.set__data(1);
        } else {
            message.gps_status.set__data(0);
        }
        message.imu_published.set__data(false); // TODO: add imu monitoring and set this accordingly
        message.emergency_status.set__data(0); // TODO: set this based on the
        // overall system status and whether we are in an emergency state or not
        status_publisher_->publish(message);
    }

    void dvl_callback(const dvl_msgs::msg::DVLDR &msg){
        if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0){
            return;
        }
        this->latest_dvl_message=msg;
    }

    void monitor_dvl(){
        if (this->get_clock()->now().seconds() - this->latest_dvl_message.header.stamp.sec > this->dvl_connection_timeout_){
             if (this->verbosity_ == 2 || (!this->dvl_conection_logged && this->verbosity_ > 0)){
                RCLCPP_ERROR(this->get_logger(), "Warning: DVL not connected.");
                this->dvl_conection_logged = true;
            }
            this->dvl_connected = false;
        } else {
            this->dvl_connected = true;
            this->dvl_conection_logged = false;
        }
        // if (this->dvl_data_invalid){
        //     this->get_logger()->error("Warning: DVL data invalid.");
        // }
    }

    void modem_callback(const seatrac_interfaces::msg::ModemStatus &msg){
        if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0){
            return;
        }
        this->latest_modem_message=msg;
    }

    void monitor_modem(){
        if (this->get_clock()->now().seconds() - this->latest_modem_message.header.stamp.sec > this->modem_connection_timeout_){
            if (this->verbosity_ == 2 || (!this->modem_conection_logged && this->verbosity_ > 0)){
                RCLCPP_ERROR(this->get_logger(), "Warning: Modem not connected.");
            }
            this->modem_conected=false;
            this->modem_conection_logged = true;
        } else {
            this->modem_conected=true;
            this->modem_conection_logged = false;
        }
    }

    void origin_callback(const geographic_msgs::msg::GeoPoint &msg) {
        this->gps_origin_lat = msg.latitude;
        this->gps_origin_lon = msg.longitude;
        this->gps_origin_alt = msg.altitude;
        this->recieved_origin = true;
    }

    void gps_callback(const gps_msgs::msg::GPSFix &msg){
        if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0){
            return;
        }
        this->latest_gps_message=msg;
        this->received_gps = true;
    }

    void monitor_gps(){
        if (!this->received_gps){
             if (this->verbosity_ == 2 || (!this->gps_conection_logged && this->verbosity_ > 0)){
                RCLCPP_ERROR(this->get_logger(), "Warning: gps fix not being publishing.");
                this->gps_conection_logged = true;
            }
        } else {
            this->gps_conection_logged = false;
        }
        if (!this->recieved_origin && ((!this->missing_origin_logged && this->verbosity_ > 0) || this->verbosity_ == 2)){
            RCLCPP_ERROR(this->get_logger(), "Warning: GPS origin not received.");
            this->missing_origin_logged = true;
        }
        else if (abs(this->latest_gps_message.latitude - this->gps_origin_lat) > this->gps_origin_dist_max_ && this->received_gps){  //if the current lat is too far from the origin lat, log an error
            RCLCPP_ERROR(this->get_logger(), "Warning: gps origin likely set incorrectly.");
        }
    }

    // void monitor_radio(){
    //     if (radio_not_connected){
    //         this->get_logger()->error("Warning: Radio not connected.");
    //     }
    // }

    // void monitor_battery(){
    //     if (battery_low){
    //         this->get_logger()->error("Warning: Battery low.");
    //     }
    // }

    rclcpp::Subscription<seatrac_interfaces::msg::ModemStatus>::SharedPtr modem_status_subscription_;
    rclcpp::Subscription<dvl_msgs::msg::DVLDR>::SharedPtr dvl_subscription_;
    rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<geographic_msgs::msg::GeoPoint>::SharedPtr gps_origin_subscription_;

    rclcpp::Publisher<cougars_interfaces::msg::SystemStatus>::SharedPtr status_publisher_;


    seatrac_interfaces::msg::ModemStatus latest_modem_message;
    bool modem_conected;
    bool modem_conection_logged;

    dvl_msgs::msg::DVLDR latest_dvl_message;
    bool dvl_connected;
    bool dvl_conection_logged;

    gps_msgs::msg::GPSFix latest_gps_message;
    bool received_gps;
    bool gps_conection_logged;

    bool recieved_origin;
    bool missing_origin_logged;
    double gps_origin_lat;
    double gps_origin_lon;
    double gps_origin_alt;

    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;


    // parameters
    int modem_connection_timeout_;
    int dvl_connection_timeout_;
    int gps_connection_timeout_;
    double gps_origin_dist_max_;
    int verbosity_;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmergencyProtocols>());
  rclcpp::shutdown();
  return 0;
}