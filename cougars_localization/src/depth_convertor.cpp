// density of water (kg/m3) + 0.8 * salinity (ppt) = density of salt water
// (kg/m3) Pawlowicz, R. (2013) Key Physical Variables in the Ocean:
// Temperature, Salinity, and Density. Nature Education Knowledge 4(4):13
#include <chrono>
#include <functional>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#define GRAVITY 9.81           // m/s^2
#define FLUID_DENSITY_BASE 997 // kg/m^3

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
    qos_profile);

/**
 * @brief A simple depth convertor node.
 * @author Nelson Durrant and Braden Meyers
 * @date December 2024
 *
 * This node subscribes to a pressure sensor topic and converts the pressure
 * data to depth data. The depth data is then published to a depth data topic.
 *
 * Subscribes:
 * - pressure/data (sensor_msgs/msg/FluidPressure)
 * Publishes:
 * - depth_data (geometry_msgs/msg/PoseWithCovarianceStamped)
 */
class DepthConvertor : public rclcpp::Node {
public:
  DepthConvertor() : Node("depth_convertor") {

    /**
     * @param water_salinity_ppt
     *
     * The salinity of the water in parts per thousand (ppt). The default value
     * is 0.0 ppt for fresh water. The value can be set to around 35.0 ppt for
     * salt water.
     */
    this->declare_parameter("water_salinity_ppt", 0.0);

    /**
     * @param fluid_pressure_atm
     *
     * The atmospheric pressure in Pascals (Pa). The default value is 87250.0 Pa
     * from our lab's field testing.
     */
    this->declare_parameter("fluid_pressure_atm", 87250.0);
    double fluid_pressure_atm =
        this->get_parameter("fluid_pressure_atm").as_double();
    average_pressure_ = fluid_pressure_atm;

    /**
     * @brief Depth publisher.
     *
     * This publisher publishes the depth data to the "depth_data" topic. It
     * uses the PoseWithCovarianceStamped message type.
     */
    depth_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "depth_data", 10);

    /**
     * @brief Pressure subscriber.
     *
     * This subscriber subscribes to the "pressure/data" topic. It uses the
     * FluidPressure message type.
     */
    pressure_subscription_ =
        this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "pressure/data", qos,
            std::bind(&DepthConvertor::pressure_callback, this, _1));

    /**
     * @brief Calibrate depth sensor service.
     *
     * This service recieves requests on the "calibrate_depth" service.
     * It uses the std_srvs/srv/Trigger service type.
     * The service will return bool if successful and a string with the offset
     * added
     */
    calibrate_service_ = this->create_service<std_srvs::srv::Trigger>(
        "calibrate_depth",
        std::bind(&DepthConvertor::calibrate_depth, this, std::placeholders::_1,
                  std::placeholders::_2));
    calibration_in_progress_ = false;

    calibration_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&DepthConvertor::timer_calibration_callback, this));

    // Start a calibration on startup of the node:
    calibration_in_progress_ = true;
    pressure_readings_.clear();
    RCLCPP_INFO(this->get_logger(),
                "Calibration started. Collecting pressure samples...");
    start_time_ = this->now();
  }

private:
  /**
   * @brief Callback function for the pressure subscription.
   *
   * This method converts the pressure data to depth data using the formulas:
   *
   * fluid density (kg/m3) = density of water (kg/m3) + 0.8 * salinity (ppt)
   *
   * depth = (fluid_pressure_atm - pressure) / (fluid_density * gravity)
   *
   * @param pressure_msg The pressure data message.
   */
  void pressure_callback(
      const sensor_msgs::msg::FluidPressure::SharedPtr pressure_msg) {

    geometry_msgs::msg::PoseWithCovarianceStamped depth_msg;
    depth_msg.header.stamp =
        pressure_msg->header.stamp; // copy exact time from the sensor message

    double fluid_density =
        FLUID_DENSITY_BASE +
        (this->get_parameter("water_salinity_ppt").as_double() * 0.8);

    depth_msg.pose.pose.position.z =
        (average_pressure_ - pressure_msg->fluid_pressure) /
        (fluid_density * GRAVITY);

    depth_publisher_->publish(depth_msg);

    if (calibration_in_progress_ && pressure_readings_.size() < 20) {
      pressure_readings_.push_back(pressure_msg->fluid_pressure);
    }
  }

  void timer_calibration_callback() {

    if (calibration_in_progress_) {
      rclcpp::Duration timeout(5, 0); // 5 seconds timeout
      if ((this->now() - start_time_) > timeout) {
        this->calculate_calibration();
        calibration_in_progress_ = false;
      } else if (pressure_readings_.size() >= 20) {
        this->calculate_calibration();
        calibration_in_progress_ = false;
      }
    }
  }

  void calibrate_depth(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    calibration_in_progress_ = true;
    pressure_readings_.clear();

    RCLCPP_INFO(this->get_logger(),
                "Calibration started. Collecting pressure samples...");

    start_time_ = this->now();

    // The service will return immediately, and the timer will handle the rest
    response->success = true;
    response->message = "Calibration started";
  }

  void calculate_calibration() {
    if (pressure_readings_.size() < 20) {
      RCLCPP_INFO(
          this->get_logger(),
          "Failed to collect sufficient pressure data for calibration.");
      return;
    }
    auto prev_pressure = average_pressure_;

    // Calculate the average pressure
    average_pressure_ = std::accumulate(pressure_readings_.begin(),
                                        pressure_readings_.end(), 0.0) /
                        pressure_readings_.size();

    // Calculate variance
    double sum_squared_diff = 0.0;
    for (const auto &pressure : pressure_readings_) {
      double diff = pressure - average_pressure_;
      sum_squared_diff += diff * diff;
    }
    double variance = sum_squared_diff / pressure_readings_.size();

    // Get the offset
    double parameter_pressure =
        this->get_parameter("fluid_pressure_atm").as_double();
    double offset = average_pressure_ - parameter_pressure;

    // Check if calibration was successful
    // Check both the offset and the variance
    // TODO make it so we can use this without hardcoded values
    // const double MAX_VARIANCE = 10000.0; // Adjust this value as needed (in
    // Pa^2) if (std::abs(offset) > 3000.0 || variance > MAX_VARIANCE) {
    //     RCLCPP_INFO(this->get_logger(), "Calibration failed. Offset: %.2f Pa,
    //     Variance: %.2f Pa^2, Calibrated Pressure: %.2f Pa",
    //                 offset, variance, average_pressure_);
    //     average_pressure_ = prev_pressure;
    //     return;
    // }

    // Calibration successful
    RCLCPP_INFO(this->get_logger(),
                "Calibration successful. Offset: %.2f Pa, Variance: %.2f Pa^2, "
                "Calibrated Pressure: %.2f Pa",
                offset, variance, average_pressure_);

    rclcpp::Parameter param("fluid_pressure_atm", average_pressure_);
    this->set_parameters({param});
  }

  // TODO: Parameter for how many samples, timeout

  // ROS objects
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      depth_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr
      pressure_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_service_;
  rclcpp::TimerBase::SharedPtr calibration_timer_;

  // Calibration variables
  std::vector<double> pressure_readings_;
  bool calibration_in_progress_;
  double average_pressure_;
  rclcpp::Time start_time_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthConvertor>());
  rclcpp::shutdown();
  return 0;
}