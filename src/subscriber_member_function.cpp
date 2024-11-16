/**
 * @file subscriber_member_function.cpp
 * @author Rishie Raj (rraj27@umd.com)
 * @brief Program to subscribe to a topic and print the message received
 * @version 0.1
 * @date 2024-11-07
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

/**
 * @brief ROS2 Node to subscribe to a topic and print the message received
 *
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Subscriber object
   *
   */
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  /**
   * @brief Callback function to print the message received
   *
   * @param msg
   */
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg->data.c_str());
  }

  // Data member to store the subscription
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
