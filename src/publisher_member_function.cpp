// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <beginner_tutorials/srv/change_str.hpp>

class PublisherandServiceNode : public rclcpp::Node {
 public:
  PublisherandServiceNode() : Node("publisher_service_node") {
    this->declare_parameter("publish_frequency", 500);
    this->message.data = "Use service to change this string";
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    service_ = this->create_service<beginner_tutorials::srv::ChangeStr>(
        "change_string",
        std::bind(&PublisherandServiceNode::change_str, this,
                  std::placeholders::_1, std::placeholders::_2));
    if (this->get_parameter("publish_frequency").as_int() == 500) {
      RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Publisher frequency has not changed, is still : "
              << this->get_parameter("publish_frequency").as_int());
    }
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(
            this->get_parameter("publish_frequency").as_int()),
        std::bind(&PublisherandServiceNode::timer_callback, this));
  }

 private:
  void timer_callback() {
    auto message = this->message;
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing : " << message.data);
    publisher_->publish(message);
  }

  void change_str(
      const std::shared_ptr<beginner_tutorials::srv::ChangeStr::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ChangeStr::Response> resp) {
    this->message.data = request->new_string;
    resp->string_change_status = request->new_string;
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Received Service Request : " << request->new_string);
  }
  std_msgs::msg::String message;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ChangeStr>::SharedPtr service_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherandServiceNode>());
  rclcpp::shutdown();
  return 0;
}
