
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "std_msgs/msg/header.hpp"
#include <cmath>

using std::string;

/**
 * @brief A ros node that publishes static transforms
 * 
 * StaticTransformPublisher
 * 
 * defines the static transformations between different coodinate frames
 * 
 * Main Coordinate Frames:
 *  - robot: the coordinate frame at the center of the robot 
 *           x: forward, y: portside, z: top of vehicle 
 *  - enu: coordinate frame at the origin
 *         x: east, y: north, z: up
 *  - ned: coodinate frame at the origin
 *         x: north, y: east, z: down
 * 
 * Sensor Coordinate Frames:
 *  - modem: the coordinate frame of the acoustic modem and its onboard imu
 *  - dvl: the coordinate frame of the dvl
 */
class StaticTransformPublisher : public rclcpp::Node {
public:

  StaticTransformPublisher() : Node("static_tf_publisher") {
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // The static transformation between robot and modem.
    // disregarding translation, the modem oriented in the same direction as the robot


    /**
     * @param modem.robot the transform from the modem to the robot position
     * The default configuration on a CougUV has no rotation and does not need translation
     */
    declare_static_tf_parameters("modem", "robot", {0.0,0.0,0.0}, {0.0,0.0,0.0,1.0});
    set_static_transform_from_param("modem", "robot", true, true);

    set_static_transform(
        "enu", "ned",
        0, 0, 0,
        std::sqrt(2)/2, std::sqrt(2)/2, 0, 0
    );

  }


  /**
   * @return namespace prefix for local coordinate frames
   * 
   * All tf transformations are published to the global /tf topic
   * To differentiate between coordinate frames of different vehicles
   * we prefix the coordinate frame name with the namespace of the vehicle.
   * 
   */
  string ns_prefix() {
    string ns = string(this->get_namespace());
    if(ns.length() <= 1) return string("");
    else return ns.substr(1)+".";
  }


  /**
   * @brief broadcasts a tf2 static transform
   * 
   * @param header_frame the frame the transform is from
   * @param child_frame the coordinate frame the transform it to
   * @param x translation in the x direction
   * @param y translation in the y direction
   * @param z translation in the z direction
   * @param qx quaternion x value
   * @param qy quaternion y value
   * @param qz quaternion z value
   * @param qw quaternion w value
   */
  void set_static_transform(
    string const& header_frame, string const& child_frame,
    float x,float y,float z,
    float qx, float qy, float qz, float qw
  ) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = header_frame;  // Parent frame
    transform.child_frame_id = child_frame;   // Child frame

    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;

    transform.transform.rotation.x = qx;
    transform.transform.rotation.y = qy;
    transform.transform.rotation.z = qz;
    transform.transform.rotation.w = qw;

    broadcaster_->sendTransform(transform);

    RCLCPP_INFO(this->get_logger(), "Defined Static Transform from %s to %s",
         header_frame.c_str(), child_frame.c_str());
  }

  /**
   * @brief declares a static tf ros parameter
   * 
   * The format is <header_frame>.<child_frame>.translation for the translation component
   * and <header_frame>.<child_frame>.orientation for the orientation component
   * 
   * @param header_frame the header frame of the transform 
   * @param child_frame the child frame of the transform (transform is child_frame wrt header_frame)
   * @param default_translation [Optional] default translation of transform
   * @param default_orientation [Optional] default orientation of transform 
   */
  void declare_static_tf_parameters(
    string const& header_frame, std::string const& child_frame
  ) {
    this->declare_parameter<std::vector<double>>(header_frame+"."+child_frame+".translation");
    this->declare_parameter<std::vector<double>>(header_frame+"."+child_frame+".orientation");
  }
  /**
   * @brief declares a static tf ros parameter
   * 
   * The format is <header_frame>.<child_frame>.translation for the translation component
   * and <header_frame>.<child_frame>.orientation for the orientation component
   * 
   * @param header_frame the header frame of the transform 
   * @param child_frame the child frame of the transform (transform is child_frame wrt header_frame)
   * @param default_translation [Optional] default translation of transform
   * @param default_orientation [Optional] default orientation of transform 
   */
  void declare_static_tf_parameters(
    string const& header_frame, std::string const& child_frame,
    std::vector<double> const& default_translation, std::vector<double> const& default_orientation
  ) {
    this->declare_parameter<std::vector<double>>(header_frame+"."+child_frame+".translation", default_translation);
    this->declare_parameter<std::vector<double>>(header_frame+"."+child_frame+".orientation", default_orientation);
  }

  /**
   * @brief sets the static transform from a ros parameter
   * 
   * sets the static transform from a parameter file using the following format:
   * ```
   * <namespace>:
   *   static_tf_publisher:
   *     <header_frame>.<child_frame>.translation: [<x>, <y>, <z>]
   *     <header_frame>.<child_frame>.orientation: [<qx>, <qy>, <qz>, <qw>]
   * ```
   * Where the translation is a vector and the orientation is a quaternion
   * 
   * @param header_frame the header frame of the transform 
   * @param child_frame the child frame of the transform (transform is child_frame wrt header_frame)
   * @param prefix_header_with_ns indicates whether or not to prefix the header with the namespace
   * @param prefix_child_with_ns indicates whether or not to prefix the child frame with the namespace
   */
  void set_static_transform_from_param(
    string const& header_frame, std::string const& child_frame,
    bool prefix_header_with_ns = false, bool prefix_child_with_ns = false
  ) {
    std::vector<double> translation;
    this->get_parameter(header_frame+"."+child_frame+".translation", translation);
    std::vector<double> orientation;
    this->get_parameter(header_frame+"."+child_frame+".orientation", orientation);

    string header_prefix = prefix_header_with_ns? ns_prefix(): string("");
    string child_prefix = prefix_child_with_ns? ns_prefix() : string("");
    set_static_transform(
      header_prefix+header_frame, child_prefix+child_frame,
      translation.at(0), translation.at(1), translation.at(2),
      orientation.at(0), orientation.at(1), orientation.at(2), orientation.at(3)
    );
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticTransformPublisher>());
  rclcpp::shutdown();
  return 0;
}