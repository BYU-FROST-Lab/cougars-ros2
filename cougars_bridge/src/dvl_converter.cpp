#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "dvl_msgs/msg/dvl.hpp"
#include "dvl_msgs/msg/dvldr.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <stdint.h>

using std::placeholders::_1;

// rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
// auto qos = rclcpp::QoS(
//     rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
//     qos_profile);

class DVLConvertor : public rclcpp::Node {
public:

  DVLConvertor() : Node("dvl_convertor") {
    // publisher_dvl_depth =
    //     this->create_publisher<std_msgs::msg::Float64>("dvl_dfb", 10);
    publisher_dvl_velocity =
        this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "dvl/velocity", 10);
    publisher_dvl_dead_reckoning =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "dvl/dead_reckoning", 10);
    subscriber_dvl_data = this->create_subscription<dvl_msgs::msg::DVL>(
        "dvl/data", 10,
        std::bind(&DVLConvertor::dvl_data_callback, this, _1));
    subscriber_dvl_position = this->create_subscription<dvl_msgs::msg::DVLDR>(
        "dvl/position", 10,
        std::bind(&DVLConvertor::dvl_pos_callback, this, _1));
  }

  void dvl_data_callback(const dvl_msgs::msg::DVL::SharedPtr msg) {

        geometry_msgs::msg::TwistWithCovarianceStamped stamped_msg;
    // This is if we copy the time when it recieves the data
    // stamped_msg.header.stamp = msg->header.stamp;

    // This should grab the time from the DVL that is using the NTP server
    float time = msg->time;
    stamped_msg.header.stamp.sec = static_cast<int32_t>(time);
    stamped_msg.header.stamp.nanosec = static_cast<uint32_t>((time - stamped_msg.header.stamp.sec) * 1e9);

    // filling in the upper left corner of the 6X6 covariance matrix
    // HANDLE CASE WHEN COVARIANCE IS EMPTY?
    int index = 0;
    double defaultValue = 0;
    for (int i = 0; i < 36; i++) {
            if (i % 6 < 3 && i < 15) {
        stamped_msg.twist.covariance[i] = msg->covariance[index];
        index++;
      } else {
        stamped_msg.twist.covariance[i] = defaultValue;
      }
    }
    
    stamped_msg.twist.twist.linear.x = msg->velocity.x;
    // negate z and y -- will this mess with covariance?
    stamped_msg.twist.twist.linear.y = -1.0 * msg->velocity.y;
    stamped_msg.twist.twist.linear.z = -1.0 * msg->velocity.z;

    // Publish the velocity only if the velocity is reported to be valid.
    if(msg->velocity_valid){
      publisher_dvl_velocity->publish(stamped_msg);
    }
    else{
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 4000, "Velocity not valid!");
    }
  }

  float degreesToRadians(float degrees) {
    return degrees * static_cast<float>(M_PI) / 180.0f;
  }

  void dvl_pos_callback(const dvl_msgs::msg::DVLDR::SharedPtr msg) {

    geometry_msgs::msg::PoseWithCovarianceStamped stamped_msg;
    stamped_msg.header.stamp = msg->header.stamp;

    stamped_msg.header.frame_id =
        "odom"; // odom is the coordinate frame of the dvl in robot_localization

    stamped_msg.pose.pose.position.x = msg->position.x;
    stamped_msg.pose.pose.position.y = -1.0 * msg->position.y;
    stamped_msg.pose.pose.position.z = -1.0 * msg->position.z;

    float yaw_deg = msg->yaw;
    float pitch_deg = msg->pitch;
    float roll_deg = msg->roll;

    // Convert to radians
    float yaw_rad = degreesToRadians(yaw_deg);
    float pitch_rad = degreesToRadians(pitch_deg);
    float roll_rad = degreesToRadians(roll_deg);

    // Option 1: use properties of quaternions
    // already tested because this was easy to do in the factor graph file
    // i*(w + xi + yj + zk)*i
    // = i*(wi - x - yk + zj)
    // = -w - xi + yj + zk
    // DVL EULER ANGLE SEQUENCE IS ZYX Intrinsic rotations
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(pitch_rad, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(roll_rad, Eigen::Vector3f::UnitX());
    Eigen::Quaternionf quaternion_R(R);
    stamped_msg.pose.pose.orientation.x = -quaternion_R.x();
    stamped_msg.pose.pose.orientation.y = quaternion_R.y();
    stamped_msg.pose.pose.orientation.z = quaternion_R.z();
    stamped_msg.pose.pose.orientation.w = -quaternion_R.w();

    // Option 2: convert with rotations. Same effect but more readable
    //  Eigen::Matrix3f R;
    //  R = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()) *
    //      Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ()) *
    //      Eigen::AngleAxisf(pitch_rad, Eigen::Vector3f::UnitY()) *
    //      Eigen::AngleAxisf(roll_rad, Eigen::Vector3f::UnitX()) *
    //      Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX());
    //  Eigen::Quaternionf quaternion_R(R);
    //  stamped_msg.pose.pose.orientation.x = quaternion_R.x();
    //  stamped_msg.pose.pose.orientation.y = quaternion_R.y();
    //  stamped_msg.pose.pose.orientation.z = quaternion_R.z();
    //  stamped_msg.pose.pose.orientation.w = quaternion_R.w();

    // Eigen::Matrix3f R;
    // R = Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ())*
    //     Eigen::AngleAxisf(pitch_rad, Eigen::Vector3f::UnitY()) *
    //     Eigen::AngleAxisf(roll_rad, Eigen::Vector3f::UnitX()) ;

    // Eigen::Quaternionf quaternion_R(R);
    // stamped_msg.pose.pose.orientation.x = quaternion_R.x();
    // stamped_msg.pose.pose.orientation.y = quaternion_R.y();
    // stamped_msg.pose.pose.orientation.z = quaternion_R.z();
    // stamped_msg.pose.pose.orientation.w = quaternion_R.w();

    // TODO: Check and tune covariance parameters
    double pvr = msg->pos_std * msg->pos_std; // variance = std squared
    double yvr = 1.0; // An estimate for the variance in yaw
    stamped_msg.pose.covariance = {
        pvr, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, pvr, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, pvr, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, yvr,
    };

    publisher_dvl_dead_reckoning->publish(stamped_msg);
  }

private:
  // publisher localizatoin pkg types
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
      publisher_dvl_velocity;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      publisher_dvl_dead_reckoning;
  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_dvl_depth;

  // subscribers - listening to dvl driver
  rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr subscriber_dvl_data;
  rclcpp::Subscription<dvl_msgs::msg::DVLDR>::SharedPtr subscriber_dvl_position;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DVLConvertor>());
  rclcpp::shutdown();
  return 0;
}