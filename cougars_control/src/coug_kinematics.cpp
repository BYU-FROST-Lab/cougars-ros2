#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "cougars_interfaces/msg/system_control.hpp"
#include "cougars_interfaces/msg/u_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
    qos_profile);

/**
 * @brief CougUV kinematics node.
 * @author Nelson Durrant
 * @date September 2024
 *
 * This node subscribes to the "controls/command" topic and modifies the
 * commands to account for thruster trim and fin offsets. The modified commands
 * are then published to the "kinematics/command" topic.
 *
 * Subscribes:
 * - controls/command (cougars_interfaces/msg/UCommand)
 *
 * Publishes:
 * - kinematics/command (cougars_interfaces/msg/UCommand)
 */
class CougKinematics : public rclcpp::Node {
public:
  /**
   * Creates a new kinematics node.
   */
  CougKinematics() : Node("coug_kinematics") {

    /**
     * @param trim_ratio
     *
     * The trim ratio is used to adjust the control commands to account for
     * thruster trim. The default value is 0.0.
     */
    this->declare_parameter("trim_ratio", 0.0);

    /**
     * @param top_fin_offset
     *
     * The offset for the top fin to make it align properly. The default value
     * is 0.0.
     */
    this->declare_parameter("top_fin_offset", 0.0);

    /**
     * @param right_fin_offset
     *
     * The offset for the right fin (from the front) to make it align properly.
     * The default value is 0.0.
     */
    this->declare_parameter("right_fin_offset", 0.0);

    /**
     * @param left_fin_offset
     *
     * The offset for the left fin (from the front) to make it align properly.
     * The default value is 0.0.
     */
    this->declare_parameter("left_fin_offset", 0.0);

    /**
     * @param fin_0_direction
     *
     * The direction of the fin 0 (top fin) reverse if getting opposite
     * behavior.
     */
    this->declare_parameter("fin_0_direction", 1);

    /**
     * @param fin_1_direction
     *
     * The direction of the fin 1 (#TODO) reverse if getting opposite behavior.
     */
    this->declare_parameter("fin_1_direction", 1);

    /**
     * @param fin_2_direction
     *
     * The direction of the fin 2 (#TODO) reverse if getting opposite behavior.
     */
    this->declare_parameter("fin_2_direction", 1);

    /**
     * @param demo_mode
     *
     * If true, the node will disable the thruster.
     * The default value is false.
     */
    this->declare_parameter("demo_mode", false);

    /**
     * @brief Kinematics thruster arm service.
     *
     * This service recieves requests on the "/arm_thruster" service.
     * It uses the std_srvs/srv/SetBool service type.
     * False will unarm the thruster and true will arm the thruster
     */
    arm_service_ = this->create_service<std_srvs::srv::SetBool>(
        "arm_thruster",
        std::bind(&CougKinematics::arm_thruster, this, std::placeholders::_1,
                  std::placeholders::_2));

    /**
     * @brief Kinematics pitch up override service.
     *
     * This service recieves requests on the "/surface" service.
     * It uses the std_srvs/srv/SetBool service type.
     * False will allow for normal control, true will override any commands and
     * pitch vehicle up
     */
    surface_service_ = this->create_service<std_srvs::srv::SetBool>(
        "surface", std::bind(&CougKinematics::surface, this,
                             std::placeholders::_1, std::placeholders::_2));

    surface_command_ = cougars_interfaces::msg::UCommand();
    surface_command_.fin[0] =
        0 * this->get_parameter("fin_0_direction").as_int() +
        this->get_parameter("top_fin_offset").as_double();
    surface_command_.fin[1] =
        -30 * this->get_parameter("fin_1_direction").as_int() +
        this->get_parameter("right_fin_offset").as_double();
    surface_command_.fin[2] =
        30 * this->get_parameter("fin_2_direction").as_int() +
        this->get_parameter("left_fin_offset").as_double();
    surface_command_.thruster = 0;

    /**
     * @brief Kinematics pitch up override service.
     *
     * This timer will ensure if the controller is not publishing, the
     * kinematics surface override will still publish. Period should be less
     * than the teensy timeout
     */
    backup_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&CougKinematics::timer_callback, this));

    /**
     * @brief Kinematics command publisher.
     *
     * This publisher publishes the commands to the "kinematics/command" topic.
     * It uses the UCommand message type.
     */
    command_publisher_ =
        this->create_publisher<cougars_interfaces::msg::UCommand>(
            "kinematics/command", 10);

    /**
     * @brief Controls command subscriber.
     *
     * This subscriber subscribes to the "controls/command" topic. It uses the
     * UCommand message type.
     */
    command_subscription_ =
        this->create_subscription<cougars_interfaces::msg::UCommand>(
            "controls/command", 10,
            std::bind(&CougKinematics::command_callback, this, _1));

    system_control_sub_ =
        this->create_subscription<cougars_interfaces::msg::SystemControl>(
            "system/status", 1,
            std::bind(&CougKinematics::system_callback, this, _1));
  }

private:
  /**
   * @brief Callback function for the controls command subscription.
   *
   * This method is called whenever a new controls command message is received.
   * It adds the declared offsets and trim ratio to the commands and publishes
   * the new modified commands.
   *
   * @param msg The received UCommand message
   */
  void command_callback(const cougars_interfaces::msg::UCommand &msg) {

    auto command = cougars_interfaces::msg::UCommand();
    command.header.stamp = msg.header.stamp;

    if (!surface_override_) {
      command.fin[0] =
          this->get_parameter("fin_0_direction").as_int() * msg.fin[0] +
          this->get_parameter("top_fin_offset").as_double() +
          this->get_parameter("trim_ratio").as_double() * msg.thruster;
      command.fin[1] =
          this->get_parameter("fin_1_direction").as_int() * msg.fin[1] +
          this->get_parameter("right_fin_offset").as_double() +
          this->get_parameter("trim_ratio").as_double() * msg.thruster;
      command.fin[2] =
          this->get_parameter("fin_2_direction").as_int() * msg.fin[2] +
          this->get_parameter("left_fin_offset").as_double() +
          this->get_parameter("trim_ratio").as_double() * msg.thruster;
    } else {
      command = surface_command_;
    }

    if (this->get_parameter("demo_mode").as_bool()) {
      command.thruster = 0;
      surface_command_.thruster = 0;
      for (auto &f : command.fin) {
        f *= 2;
      }
    } else if (!arm_thruster_) {
      command.thruster = 0;
      surface_command_.thruster = 0;
    } else {
      command.thruster = msg.thruster;
      surface_command_.thruster = msg.thruster;
      // TODO: Should we save the last thruster command and use it in the timer
      // callback?
    }

    command_publisher_->publish(command);
  }

  void
  system_callback(const cougars_interfaces::msg::SystemControl::SharedPtr msg) {
    // Set the boolean to the requested value
    arm_thruster_ = msg->thruster_arm.data;
    RCLCPP_INFO(this->get_logger(),
                arm_thruster_ ? "Thruster armed!" : "Thruster unarmed!");

    if (!arm_thruster_) {
      surface_command_.thruster = 0;
    }
  }

  void
  arm_thruster(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    // Set the boolean to the requested value
    arm_thruster_ = request->data;
    response->success = true;
    response->message = arm_thruster_ ? "Thruster armed!" : "Thruster unarmed!";
    RCLCPP_INFO(this->get_logger(), response->message.c_str());

    if (!arm_thruster_) {
      surface_command_.thruster = 0;
    }
  }

  void surface(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    // Set the boolean to the requested value
    surface_override_ = request->data;
    response->success = true;
    response->message = surface_override_ ? "Fin Override!" : "Fin actuation!";
    RCLCPP_INFO(this->get_logger(), response->message.c_str());

    command_publisher_->publish(surface_command_);
  }

  void timer_callback() {
    // Publish the fins up command
    if (surface_override_) {
      command_publisher_->publish(surface_command_);
    }
  }

  bool arm_thruster_ = false;
  bool surface_override_ = false;

  cougars_interfaces::msg::UCommand surface_command_;

  // ROS objects
  rclcpp::TimerBase::SharedPtr backup_timer_;
  rclcpp::Publisher<cougars_interfaces::msg::UCommand>::SharedPtr
      command_publisher_;
  rclcpp::Subscription<cougars_interfaces::msg::UCommand>::SharedPtr
      command_subscription_;
  rclcpp::Subscription<cougars_interfaces::msg::SystemControl>::SharedPtr
      system_control_sub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr surface_service_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CougKinematics>());
  rclcpp::shutdown();
  return 0;
}
