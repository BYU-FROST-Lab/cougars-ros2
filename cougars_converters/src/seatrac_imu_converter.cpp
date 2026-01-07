
#include <cmath>
#include <chrono>
#include <seatrac_interfaces/msg/modem_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#define SQRT_OF_2_OVER_2 0.70710678118
#define PI 3.14159265359 
#define DEGREES_TO_RADIANS PI/180.0

//From experimentation, 1 G, equivelant to 9.81 m/s^2, is about 250 in raw accelerometer units
#define ACC_UNITS_TO_METERS_PER_SECOND_SQUARED 9.80665 / 250.0

using std::string;
using namespace std::chrono_literals;

/**
 * @brief publishes an imu message using data from the seatrac modem
 * @author Clayton Smith
 * @date Febuary 2025
 *
 * This node converts imu data from the seatrac usbl beacon into
 * an imu message in the coodinate frame of the vehicle.
 *
 * Subscribes:
 * - modem_status (seatrac_interfaces/msg/ModemStatus)
 * Publishes:
 * - seatrac/imu/data (sensor_msgs::msg::Imu)
 * - /tf <"ned"->"modem"> (geometry_msgs/msg/TransformStamped)
 * - imu_accel_marker (visualization_msgs/msg/Marker)
 * - imu_ang_vel_marker (visualization_msgs/msg/Marker)
 */
class SeatracAHRSConverter : public rclcpp::Node {
public:
  SeatracAHRSConverter() : Node("seatrac_ahrs_converter") {
    /**
     * @param magnetic_declination
     * https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?
     */
    this->declare_parameter("magnetic_declination",
                            10.7); // 10.70° E for Utah Lake
    magnetic_declination = this->get_parameter("magnetic_declination").as_double();

    /**
     * @param publish_rviz_markers
     * Publishes visual markers that rviz can use to show the acceleration and angular velocity
     * output by the seatrac imu. Usefull for debugging.
     */
    this->declare_parameter<bool>("publish_rviz_markers", false);
    publish_rviz_markers_ = this->get_parameter("publish_rviz_markers").as_bool();

    if(publish_rviz_markers_) {
      accel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("imu_accel_marker", 10);
      ang_vel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("imu_ang_vel_marker", 10);
      RCLCPP_INFO(this->get_logger(), "seatrac_ahrs_converter set to publish imu_accel_marker and imu_ang_vel_marker");
      RCLCPP_INFO(this->get_logger(), "to visualize the acceleration and angular velocity in rviz");
    }

    modem_subscriber_ =
        this->create_subscription<seatrac_interfaces::msg::ModemStatus>(
            "modem_status", 10,
            std::bind(&SeatracAHRSConverter::modem_callback, this,
                      std::placeholders::_1));

    modem_imu_pub_ =
        this->create_publisher<sensor_msgs::msg::Imu>("seatrac/imu/data", 10);

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  }

private:


  /**
   * @brief callback on modem status message
   * 
   * First, broadcasts the transform from ned to modem to /tf
   * Second, uses that transform to make an Imu message in the robot's coordinate frame
   *
   * @param msg modem status ros message
   */
  void modem_callback(const seatrac_interfaces::msg::ModemStatus::SharedPtr msg) {

    double yaw   = (M_PI / 180.0) * (0.1 * msg->attitude_yaw + magnetic_declination);
    double pitch = (M_PI / 180.0) * 0.1 * msg->attitude_pitch;
    double roll  = (M_PI / 180.0) * 0.1 * msg->attitude_roll;

    //==========================//
    // tf transform message     //
    //==========================//

    geometry_msgs::msg::TransformStamped t;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "ned";
    t.child_frame_id = ns_prefix()+"modem";

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    //==========================//
    //   publish as modem_imu   //
    //==========================//

    try {

      auto modem_imu = std::make_shared<sensor_msgs::msg::Imu>();

      modem_imu->header.stamp = msg->header.stamp;
      modem_imu->header.frame_id = ns_prefix()+"robot";

      geometry_msgs::msg::TransformStamped tf_enu_to_robot =
          tf_buffer_->lookupTransform("enu", ns_prefix()+"robot", tf2::TimePointZero);

      modem_imu->orientation.x = tf_enu_to_robot.transform.rotation.x;
      modem_imu->orientation.y = tf_enu_to_robot.transform.rotation.y;
      modem_imu->orientation.z = tf_enu_to_robot.transform.rotation.z;
      modem_imu->orientation.w = tf_enu_to_robot.transform.rotation.w;

      // The gyroscope is mounted with a 45 degree offset from the modem frame
      // So a linear transformation is needed. Determined experimentally.
      tf2::Vector3 angular_velocity(
          DEGREES_TO_RADIANS*SQRT_OF_2_OVER_2*(msg->gyro_y - msg->gyro_x),
          DEGREES_TO_RADIANS*SQRT_OF_2_OVER_2*(msg->gyro_x + msg->gyro_y),
          -DEGREES_TO_RADIANS*msg->gyro_z
      );

      // The accelerometer is mounted with a 45 degree offset from the modem frame
      // so a linear transformation is need. Determined experimentally.
      tf2::Vector3 linear_acceleration(
          ACC_UNITS_TO_METERS_PER_SECOND_SQUARED * SQRT_OF_2_OVER_2*(msg->acc_x - msg->acc_y),
          -ACC_UNITS_TO_METERS_PER_SECOND_SQUARED * SQRT_OF_2_OVER_2*(msg->acc_x + msg->acc_y),
          ACC_UNITS_TO_METERS_PER_SECOND_SQUARED * msg->acc_z
      );

      geometry_msgs::msg::TransformStamped tf_modem_to_robot =
          tf_buffer_->lookupTransform(ns_prefix()+"modem", ns_prefix()+"robot", tf2::TimePointZero);

      // Extract rotation
      tf2::Quaternion rotation(
        tf_modem_to_robot.transform.rotation.x,
        tf_modem_to_robot.transform.rotation.y,
        tf_modem_to_robot.transform.rotation.z,
        tf_modem_to_robot.transform.rotation.w);
      tf2::Transform tf_transform(rotation);
      tf2::Matrix3x3 rot_matrix = tf_transform.getBasis();

      //transform angular velocities and accelerations
      tf2::Vector3 robot_angular_vel = rot_matrix * angular_velocity;
      tf2::Vector3 robot_linear_accel = rot_matrix * linear_acceleration;

      //copy values to imu message
      modem_imu->angular_velocity.x = robot_angular_vel.x();
      modem_imu->angular_velocity.y = robot_angular_vel.y();
      modem_imu->angular_velocity.z = robot_angular_vel.z();

      modem_imu->linear_acceleration.x = robot_linear_accel.x();
      modem_imu->linear_acceleration.y = robot_linear_accel.y();
      modem_imu->linear_acceleration.z = robot_linear_accel.z();

      // Publish the IMU message
      modem_imu_pub_->publish(*modem_imu);

      if(publish_rviz_markers_) {
        publishVectorMarker(modem_imu->linear_acceleration, "acceleration", accel_pub_, 1.0, 0.0, 0.0);
        publishVectorMarker(modem_imu->angular_velocity, "angular_velocity", ang_vel_pub_, 0.0, 1.0, 0.0);
      }

    } catch(const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), 
                           *this->get_clock(), 5s,
                           "Transform not available, Waiting for Transform: %s", ex.what());
    }

  }

  /**
   * @brief method to publish an rviz arrow marker
   * 
   * Used to visualize the acceleration and angular velocity vectors in rviz
   * 
   * @param vec the vector to publish
   * @param name
   * @param publisher the publisher to use (either accel_pub_ or ang_vel_pub_)
   * @param r red color intensity
   * @param g green color intensity
   * @param b blue color intensity
   */
  void publishVectorMarker(
    const geometry_msgs::msg::Vector3& vec,
    const std::string& name,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
    float r, float g, float b) 
  {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = ns_prefix()+"robot"; // Change if necessary
    marker.header.stamp = this->now();
    marker.ns = name;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Scale
    marker.scale.x = .01;
    marker.scale.y = .02;// * 0.1;
    marker.scale.z = .02;// * 0.1;

    // Start point (origin)
    geometry_msgs::msg::Point start;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;

    // End point (scaled vector)
    geometry_msgs::msg::Point end;
    end.x = vec.x*.02;
    end.y = vec.y*.02;
    end.z = vec.z*.02;

    marker.points.push_back(start);
    marker.points.push_back(end);

    // Color
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    publisher->publish(marker);
  }

  /**
   * @return namespace prefix for local coordinate frames
   * 
   * All tf transformations are published to the global /tf topic
   * To differentiate between coordinate frames of different vehicles
   * we prefix the coordinate frame name with the namespace of the vehicle
   * 
   */
  string ns_prefix() {
    string ns = string(this->get_namespace());
    if(ns.length() <= 1) return string("");
    else return ns.substr(1)+".";
  }

  bool publish_rviz_markers_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr accel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ang_vel_pub_;

  rclcpp::Subscription<seatrac_interfaces::msg::ModemStatus>::SharedPtr
      modem_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr modem_imu_pub_;
  double magnetic_declination;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SeatracAHRSConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}