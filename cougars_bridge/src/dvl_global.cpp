#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class DvlTransformNode : public rclcpp::Node {
public:
    DvlTransformNode() : Node("dvl_transform_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // Publisher for the transformed data
        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("dvl/dr_global", 10);

        // Subscriber for the input data
        sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "dvl/dead_reckoning", 10,
            std::bind(&DvlTransformNode::poseCallback, this, std::placeholders::_1));
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        geometry_msgs::msg::PoseWithCovarianceStamped transformed_msg = *msg;

        try {
            // Get the latest transformation
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_.lookupTransform("world", "odom", tf2::TimePointZero);

            // Transform the pose
            tf2::doTransform(msg->pose, transformed_msg.pose, transform_stamped);

            // Update header
            transformed_msg.header.frame_id = "world";
        } catch (const tf2::TransformException &ex) {
            // If no valid transform, just keep the original pose
            RCLCPP_WARN_ONCE(this->get_logger(), "Transform not available, Waiting for Transform: %s", ex.what());
        }

        // Publish the transformed or original message
        pub_->publish(transformed_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DvlTransformNode>());
    rclcpp::shutdown();
    return 0;
}
