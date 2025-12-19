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
#include "rcl_interfaces/srv/get_parameters.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
    qos_profile);


static double depth_val;
static bool back_near_surface;
static bool surfacing_then_disarm;
class EmergencyProtocols : public rclcpp::Node
{


public:
  //////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////// CONSTRUCTOR ///////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////
  EmergencyProtocols()
  : Node("emergency_protocols")
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////// PARAMETERS /////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////

    // set all parmeters in yaml file

    // controlling what you want to monitor:
    // if these flags are set to true, then emergency
    // handling will occur in any relevant subscribers
  
    this->declare_parameter("monitor_leak", true);
    this->declare_parameter("monitor_low_battery", true);
    this->declare_parameter("monitor_moos", true);
    this->declare_parameter("monitor_factor_graph", true);
    this->declare_parameter("monitor_depth", true);
    this->declare_parameter("monitor_controls", true);
    this->declare_parameter("monitor_heartbeat", true);
    this->declare_parameter("monitor_waypoint_progress", true);
    this->declare_parameter("monitor_mission_timout", true);
    this->declare_parameter("monitor_dive_ability", true);
    this->declare_parameter("monitor_collision", true);


    // safety parameters (voltage, current, depth, etc.)
    this->declare_parameter("deepest_safe_depth", -2.0); // meters
    this->declare_parameter("critical_voltage", 14.0); // volts
    this->declare_parameter("modem_message_timeout", 2);
    this->declare_parameter("dvl_message_timeout",2);
    this->declare_parameter("dvl_position_stddev_threshold",5.0);
    this->declare_parameter("gps_sat_num_threshold",4);
    this->declare_parameter("gps_origin_check",false);
    this->declare_parameter("origin_gps_msgs",3);
    this->declare_parameter("origin_gps_threshold_alt",100); 
    this->declare_parameter("origin_gps_threshold",.5); //around 35 miles
    //  TODO: add more safety parameters as needed


    //////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////// SUBSCRIPTIONS and Timers ///////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////


    depth_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("depth_data", 10, std::bind(&EmergencyProtocols::depth_callback, this, _1));
    leak_subscription_ = this->create_subscription<sensor_msgs::msg::FluidPressure>("leak/data", 1, std::bind(&EmergencyProtocols::leak_callback, this, _1));
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("modem_imu", 1, std::bind(&EmergencyProtocols::imu_callback, this, _1));
    battery_subscription_ = this->create_subscription<sensor_msgs::msg::BatteryState>("battery/data", 1, std::bind(&EmergencyProtocols::battery_callback, this, _1));
    gps_subscription_ = this->create_subscription<gps_msgs::msg::GPSFix>("extended_fix", 1, std::bind(&EmergencyProtocols::gps_fix_callback, this, _1));
    dvl_subscription_ = this->create_subscription<dvl_msgs::msg::DVLDR>("dvl/position",1,std::bind(&EmergencyProtocols::dvl_callback, this, _1));
    modem_subscription_ = this->create_subscription<seatrac_interfaces::msg::ModemStatus>("modem_status",1,std::bind(&EmergencyProtocols::modem_callback,this,_1));
    //         qos,std::bind(&EmergencyProtocols::factor_graph_callback, this, _1));
    // vehicle_status_moos_subscription_ = this->create_subscription<cougars_interfaces::msg::VehicleStatus>("vehicle_status", 
    //         qos, std::bind(&EmergencyProtocols::moos_status_callback, this, _1));


    command_subscription_ =
    this->create_subscription<cougars_interfaces::msg::UCommand>(
        "controls/command", 10,
        std::bind(&EmergencyProtocols::command_callback, this, _1));


    status_publisher_ = this->create_publisher<cougars_interfaces::msg::SystemStatus>("safety_status", 10);

    this->okay = true;
    this->init = false;
    this->imuPub=false;
    this->gps_count=0;
    this->gps_origin_lat=0;
    this->gps_origin_lon=0;
    this->gps_origin_alt=0;
    this->gps_bad_origin=0;
    if(this->get_parameter("gps_origin_check").as_bool()){
      grab_gps_params();
    }
  
    depth_val = 1.0;
    back_near_surface = false;
    surfacing_then_disarm = false;
    std_msgs::msg::Header defaultHeader = std_msgs::msg::Header();
    builtin_interfaces::msg::Time defaultTime =builtin_interfaces::msg::Time();
    defaultTime.set__sec(0);
    defaultHeader.set__stamp(defaultTime);
    latest_dvl_message.set__header(defaultHeader);
    latest_gps_message.set__header(defaultHeader);
    latest_modem_message.set__header(defaultHeader);


    // status publisher
    // TODO: Add safety status publisher,


    // timers
    monitor_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(3000),
        std::bind(&EmergencyProtocols::monitor_timer_callback, this));

    surface_waiter_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(5000),
          std::bind(&EmergencyProtocols::surface_waiter_timer_callback_, this));
    handle_factor_graph_ = this->create_wall_timer(
        std::chrono::milliseconds(5000),
        std::bind(&EmergencyProtocols::handle_factor_graph_callback, this));


  
    
    /// init variables
    counter = 0;

    // vector to store important topics ahd whether they have publishers or not
    // topics_publishing.insert({{"smoothed_output", true}, 
    //                           {"battery/data", true},
    //                           {"leak/data", true}, 
    //                           {"depth/data", true},
    //                           {"modem_imu", true},
    //                           {"desired_depth", true},
    //                           {"desired_heading", true},
    //                           {"desired_speed", true},
    //                           {"vehicle_status", true},
    //                           {"kinematics/command", true},
    //                           {"controls/command", true},
    //                           {"gps_odom", true},
    //                           {"extended_fix", true},
    //                           {"dvl_dead_reckoning", true}});


    topics_publishing.insert({{"battery/data", true},
                              {"leak/data", true}, 
                              {"depth_data", true},
                              {"modem_status", true},
                              {"dvl/dead_reckoning", true},
                              {"gps_odom", true},
                              {"extended_fix", true},
                              {"modem_imu",true}});
  

    //////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// INIT EMERGENCY REQUESTS ///////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////

    
    ////////////////////////////
    // (dis)arm thruster request
    ////////////////////////////
    // in response to:
    //      * leak 
    //      * low battery
    //      * TODO: add more as needed
    arm_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    arm_thruster_client = this->create_client<std_srvs::srv::SetBool>("arm_thruster");
    while (!arm_thruster_client->wait_for_service(1s)){
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        break;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    ///////////////////////////////////
    // override fins to surface request
    ///////////////////////////////////
    // in response to:
    //      * too deep  
    //      * TODO: add more as needed
    surface_fin_override_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    surface_fin_override_client = this->create_client<std_srvs::srv::SetBool>("surface");
    while (!surface_fin_override_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        break;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }


    ///////////////////////////////////
    // TODO: add other requests as necessary
    ///////////////////////////////////
    // in response to:
    //      * TODO: add more 
    //      * ...



  } // end constructor

  

 private:



void surface(){
  if (this->init){
    
    RCLCPP_INFO(this->get_logger(), "Surface");
    this->okay = false;
    surface_fin_override_request->data = true; // true to override
    send_set_bool_req(surface_fin_override_request, surface_fin_override_client);
  } 
}

void disarm(){
  if (this->init){
    RCLCPP_INFO(this->get_logger(), "Disarm");
    this->okay = false;
    arm_request->data = false; // make false to diarm the thruster
    send_set_bool_req(arm_request, arm_thruster_client);
  }
}

void surface_then_disarm(){
      surfacing_then_disarm = true;
      surface();
}

  

//////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////  CB's ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////


// monitor timer helpers


bool update_publishers(){

  bool all_topics_have_publishers = true;

  // look at all topics of interest
  for (auto itr0 : topics_publishing){

    bool topic_has_publishers = false;

    // see if everything in topics_publishing is indeed being published to
    for(auto itr : get_topic_names_and_types()){
      if(itr.first.find(itr0.first) != std::string::npos){
        if (count_publishers(itr.first) > 0){
          topic_has_publishers = true;
        }
      }
    }
    topics_publishing[itr0.first.c_str()] = topic_has_publishers;
    if (!topic_has_publishers){
      all_topics_have_publishers = false;
      this->okay = false;
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "%s has no publishers!", itr0.first.c_str());
    }
  }

  return all_topics_have_publishers;

}


////////////////////////////////////
//  Monitoring Publishers 
//  to important topics
///////////////////////////////////

  // this timer looks through all topics on the network and checks for crucial topics such as 
  // smoothed_output, leak/data, etc. Key topics are in topics_publishing map
  void monitor_timer_callback() {
    auto message = cougars_interfaces::msg::SystemStatus();
    message.dvl_status.set__data(0);
    message.modem_status.set__data(0);
    message.gps_status.set__data(0);
    message.imu_published.set__data(this->imuPub);
    if (counter < 1){ //dont run the first time
      counter++;
    }
    else{
      if(!update_publishers()){

        this->okay = false;
        // handle topics 
        if (!topics_publishing["modem_status"]){
          surface_then_disarm();
        }

      }
      if(latest_dvl_message.header.stamp.sec!=0){//checking if this is a real message, not just default
        if(((this->get_clock()->now().seconds()-latest_dvl_message.header.stamp.sec)>this->get_parameter("dvl_message_timeout").as_int())){
          this->okay = false;
          message.dvl_status.set__data(message.dvl_status.data|1);
        } 
        if(latest_dvl_message.pos_std>this->get_parameter("dvl_position_stddev_threshold").as_double()){
          this->okay=false;
          message.dvl_status.set__data(message.dvl_status.data|(1>>1));
        }
      }
      if(latest_modem_message.header.stamp.sec!=0){ //checking if this is a real message, not just default
        if(((this->get_clock()->now().seconds()-latest_modem_message.header.stamp.sec)>this->get_parameter("modem_message_timeout").as_int())){
          this->okay = false;
          message.modem_status.set__data(1);
          surface_then_disarm();
        }
      }
      if(gps_bad_origin&&this->get_parameter("gps_origin_check").as_bool()){
        message.gps_status.set__data(2);
        this->okay=false;
      }
      if(latest_gps_message.status.satellites_used<(this->get_parameter("gps_sat_num_threshold").as_int())){
        //not going to set okay to false because this is expected during the mission
        message.gps_status.set__data(message.gps_status.data|1);
      }
    }

    
    if(okay){
      message.emergency_status.set__data(0);
    } else if(back_near_surface|| !okay){
      message.emergency_status.set__data(2); 
    }else if(surfacing_then_disarm){
      message.emergency_status.set__data(1);
    }
    message.emergency_status.set__data(okay);
    status_publisher_->publish(message);

  }
  void handle_gps_param_response(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future){
    auto response = future.get();
    if (response->values.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No parameter values returned!");
      return;
    }
   this->gps_origin_lat = response->values[0].double_value; //test which value is which
   this->gps_origin_lon = response->values[1].double_value;
   this->gps_origin_alt = response->values[2].double_value;
  }
  void grab_gps_params(){
        // Create the client for the target node's parameter service
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr client_ = this->create_client<rcl_interfaces::srv::GetParameters>("/gps_odom/get_parameters");

    // Wait for the service to be available
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "waiting for parameters from gps odom");
    }

    // Create the request
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("origin.latitude");
    request->names.push_back("origin.longitude");
    request->names.push_back("origin.altitude");

    // Send the request asynchronously
    auto future = client_->async_send_request(request,
      std::bind(&EmergencyProtocols::handle_gps_param_response, this, std::placeholders::_1));
  
  }
  void gps_fix_callback(const gps_msgs::msg::GPSFix &msg){
    if(this->get_parameter("gps_origin_check").as_bool()){
    if(this->gps_count<this->get_parameter("origin_gps_msgs").as_int()){
      this->gps_count++;
      if(gps_origin_lat!=0 && gps_origin_lon!=0 && gps_origin_alt!=0){ //params grabbed
        if(abs(msg.longitude-gps_origin_lon>this->get_parameter("origin_gps_threshold").as_double()) || abs(msg.altitude-gps_origin_alt>this->get_parameter("origin_gps_threshold_alt").as_int()) || abs(msg.latitude-gps_origin_lat)>this->get_parameter("origin_gps_threshold").as_double()){
            RCLCPP_ERROR(this->get_logger(), "GPS origin likely set incorrectly");
            this->gps_bad_origin=1;
        }

      }
    }
  }
    latest_gps_message=msg;
  }


  void dvl_callback(const dvl_msgs::msg::DVLDR &msg){
    latest_dvl_message=msg;
  }
  void modem_callback(const seatrac_interfaces::msg::ModemStatus &msg){
    latest_modem_message=msg;
  }

  void command_callback(const cougars_interfaces::msg::UCommand &msg) {
      this->init = true;
  }
  void imu_callback(const sensor_msgs::msg::Imu &msg){
    this->imuPub=true;
  }
  void surface_waiter_timer_callback_(){

    if(surfacing_then_disarm && back_near_surface){
      disarm();
    }
  }



///////////////////////////////////
//  Factor Graph Emergency
///////////////////////////////////
//  Requests to be sent:
//      * surface_fin_override
//      * return_to_home 


  void handle_factor_graph_callback(){
    if (this->get_parameter("monitor_factor_graph").as_bool()){
      // handle factor graph emergencies here
      // TODO: discuss how we handle this
      // surface
      // disarm thruster
      // restart factor graph

      // if(!topics_publishing["smoothed_output"]){

      // }
    }
  }



///////////////////////////////////
//  Depth Emergency
///////////////////////////////////
//  Requests to be sent:
//      * surface_fin_override 

void depth_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg){
  if (this->get_parameter("monitor_depth").as_bool()){
    depth_val = msg.pose.pose.position.z;

    if(depth_val > -0.1){
      back_near_surface = true;
    }

    // RCLCPP_INFO(this->get_logger(), "%f\n" , depth_val);

    // if the current depth is too deep, surface
    if (depth_val < this->get_parameter("deepest_safe_depth").as_double()){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "TOO DEEP ERROR.");
        RCLCPP_INFO(this->get_logger(), "Emergency request being sent in response to leak");
        surface_then_disarm();
    }

    // TODO: put any other cases we may deduce from msg
    // ... 
    // ...
  }
}



///////////////////////////////////
// Battery Emergency
///////////////////////////////////
//  Requests to be sent:
//      * disarm thruster 

void battery_callback(const sensor_msgs::msg::BatteryState &msg){

  if (this->get_parameter("monitor_low_battery").as_bool()){
    double curr_volt = msg.voltage;
    if (curr_volt < this->get_parameter("critical_voltage").as_double()){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "LOW VOLTAGE ERROR.");


      RCLCPP_INFO(this->get_logger(), "Emergency request being sent in response to voltage");
      surface_then_disarm();
    }

    // TODO: put any other cases we may deduce from msg
    // ... 
    // ...


  }
}



///////////////////////////////////
// Leak Emergency
///////////////////////////////////
//  Requests to be sent:
//      * disarm thruster 

void leak_callback(const sensor_msgs::msg::FluidPressure &msg){

  if (this->get_parameter("monitor_leak").as_bool()){

    if (msg.fluid_pressure > 0.0){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "LEAK ERROR.");

      RCLCPP_INFO(this->get_logger(), "Emergency request being sent in response to leak");
      surface_then_disarm();

    }

    
      

  }
}

///////////////////////////////////
// MOOS Emergency
///////////////////////////////////
//TODO: handle MOOS/waypoint problems here


//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// SEND REQUEST HELPER //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

// works just for SetBool requests
 void send_set_bool_req(std::shared_ptr<std_srvs::srv::SetBool::Request> request, 
                        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client){
    if (client->service_is_ready()){
      // put in a callback function
      auto  result = client->async_send_request(request,
      [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Service response: %s", response->message.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        }
    );
    }
    else{
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "service not ready");
    }

  }

 


//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// REQUESTS & CLIENTS ///////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

  // requests  TODO: put more requests here

  std::shared_ptr<std_srvs::srv::SetBool::Request>  arm_request;
  std::shared_ptr<std_srvs::srv::SetBool::Request>  surface_fin_override_request;


  // emergency service slients, TODO: add more service clients
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_thruster_client; 
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr surface_fin_override_client;



//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// SUBSCRIPTION/TIMER DECLARATIONS //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr leak_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr smoothed_output_subscription_;
//   rclcpp::Subscription<cougars_interfaces::msg::VehicleStatus>::SharedPtr vehicle_status_moos_subscription_;
  rclcpp::Subscription<cougars_interfaces::msg::UCommand>::SharedPtr command_subscription_;
  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr gps_subscription_;
  rclcpp::Subscription<dvl_msgs::msg::DVLDR>::SharedPtr dvl_subscription_;
  rclcpp::Subscription<seatrac_interfaces::msg::ModemStatus>::SharedPtr modem_subscription_;



  //publisher
  rclcpp::Publisher<cougars_interfaces::msg::SystemStatus>::SharedPtr status_publisher_;

  // timers 
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  rclcpp::TimerBase::SharedPtr surface_waiter_timer_;
  rclcpp::TimerBase::SharedPtr handle_factor_graph_;
  int counter;

  bool okay;
  bool init;
  bool imuPub;
  int gps_count;
  float gps_origin_lat,gps_origin_lon,gps_origin_alt;
  bool gps_bad_origin;
  //messages
  gps_msgs::msg::GPSFix latest_gps_message=gps_msgs::msg::GPSFix();
  dvl_msgs::msg::DVLDR latest_dvl_message=dvl_msgs::msg::DVLDR();
  seatrac_interfaces::msg::ModemStatus latest_modem_message;
   // topic name vector
  std::map<std::string, bool> topics_publishing;
  
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmergencyProtocols>());
  rclcpp::shutdown();

  return 0;
}
