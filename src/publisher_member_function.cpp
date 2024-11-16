/**
 * @file publisher_member_function.cpp
 * @author Rishie Raj (rraj27@umd.com)
 * @brief Program for a ROS2 node that publishes a string message and acts a
 * service to change the message
 * @version 0.1
 * @date 2024-11-07
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <beginner_tutorials/srv/change_str.hpp>


/**
 * @brief ROS2 Node that publishes a string message and acts a service to change
 * the message
 *
 */
class PublisherandServiceNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Node object and instantiate a publisher and service
   *
   */
  PublisherandServiceNode() : Node("publisher_service_node") {
    this->declare_parameter("publish_frequency", 500);
    this->message.data = "Use service to change this string";
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    service_ = this->create_service<beginner_tutorials::srv::ChangeStr>(
        "change_string",
        std::bind(&PublisherandServiceNode::change_str, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Log a warning if the publish frequency is set to default
    if (this->get_parameter("publish_frequency").as_int() == 500) {
      RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Publisher frequency has not changed, is still : "
              << this->get_parameter("publish_frequency").as_int());
    }

    // Log a fatal error if the publish frequency is set very high
    if (this->get_parameter("publish_frequency").as_int() > 5000) {
      RCLCPP_FATAL_STREAM(
          this->get_logger(),
          "Publisher frequency is set very high and may cause issues!");
      rclcpp::shutdown();
      return;
    }

    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Create a timer to publish the message at the specified frequency
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(
            this->get_parameter("publish_frequency").as_int()),
        std::bind(&PublisherandServiceNode::timer_callback, this));
  }

 private:
  /**
   * @brief Callback function for the timer to publish the message
   *
   */
  void timer_callback() {
    auto message = this->message;
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing : " << message.data);
    publisher_->publish(message);
    this->publish_talk_transform();
  }

  /**
   * @brief Callback function for the service to change the message
   *
   * @param request Request message containing the new string
   * @param resp Response message containing the status of the change
   */
  void change_str(
      const std::shared_ptr<beginner_tutorials::srv::ChangeStr::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ChangeStr::Response> resp) {
    if (request->new_string.empty()) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Received empty string! Cannot update message.");
      resp->string_change_status =
          "Failed to change string: Received empty string.";
    } else {
      this->message.data = request->new_string;
      resp->string_change_status = request->new_string;
      RCLCPP_DEBUG_STREAM(this->get_logger(),
                          "Received Service Request : " << request->new_string);
    }
  }

  /**
   * @brief Function to publish a static transform for the talker frame
   *
   */
  void publish_talk_transform() {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";

    t.transform.translation.x = 0.1;
    t.transform.translation.y = 0.1;
    t.transform.translation.z = 0.1;
    tf2::Quaternion q;
    q.setRPY(0.1, 0.1, 0.1);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }

  // Data members to store the message, publisher and service
  std_msgs::msg::String message;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ChangeStr>::SharedPtr service_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherandServiceNode>());
  rclcpp::shutdown();
  return 0;
}
