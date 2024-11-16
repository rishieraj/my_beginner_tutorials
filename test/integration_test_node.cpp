// Copyright 2023 Nick Morales.
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
//
// # CHANGES:
//
// 2024-10-31, Tommy Chang
//     - Renamed the first test case to "service_test".
//     - Added a second test, "talker_test".
//     - Rewrite the first test case to use wait_for_service().
//     - Use modern ROS2 syntax
//     - Use Catch2 Fixture

/**
 * @file
 * @brief Integration test node for beginner_tutorials package.
 *
 * This file contains the integration test node for the beginner_tutorials
 * package. The integration test node is responsible for testing the
 * functionality of the nodes in the beginner_tutorials package.
 */
#include <ctime>
#include <memory>
#include <functional>

#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <catch_ros2/catch_ros2.hpp>
#include <beginner_tutorials/srv/change_str.hpp>

using std_msgs::msg::String;

////////////////////////////////////////////////
// Define Fixture
////////////////////////////////////////////////
auto Logger = rclcpp::get_logger("");  // create an initial Logger

class MyTestsFixture {
 public:
  MyTestsFixture() {
    /**
     * 1.) Create the node that performs the test. (aka Integration test node):
     */
    testerNode = rclcpp::Node::make_shared("IntegrationTestNode1");
    Logger =
        testerNode
            ->get_logger();  // make sure message will appear in rqt_console

    /**
     * 2.) Declare a parameter for the duration of the test:
     */
    testerNode->declare_parameter<double>("test_duration");

    /**
     * 3.) Get the test duration value:
     */
    TEST_DURATION = testerNode->get_parameter("test_duration")
                        .get_parameter_value()
                        .get<double>();
    RCLCPP_INFO_STREAM(Logger, "Got test_duration =" << TEST_DURATION);
  }

  ~MyTestsFixture() {}

 protected:
  double TEST_DURATION;
  rclcpp::Node::SharedPtr testerNode;
};

////////////////////////////////////////////////
// Test Case 1
////////////////////////////////////////////////

/* In this test case, the node under test (aka Auxiliary test node)
   is a service server, which got launched by the launcher.

   We will ceate a service client as part of the node performing the
   test (aka Integration test node).  And the test simply checks if
   the service is available within the duration of the test. */

TEST_CASE_METHOD(MyTestsFixture, "test service server", "[service]") {
  /**
   * 4.) Now, create a client for the specific service we're looking for:
   */
  auto client = testerNode->create_client<beginner_tutorials::srv::ChangeStr>(
      "change_string");
  RCLCPP_INFO_STREAM(Logger, "'change_string' client created");

  /**
   * 5.) Finally do the actual test:
   */
  std::time_t start_time = std::time(nullptr);  // Record start time
  bool service_found = false;
  double duration = 0;  // Duration in seconds
  RCLCPP_INFO_STREAM(Logger, "Performing Test...");

  // Timeout value in seconds
  const int timeout_seconds = static_cast<int>(TEST_DURATION);

  // Wait for the service
  if (client->wait_for_service(std::chrono::seconds(timeout_seconds))) {
    duration = std::difftime(std::time(nullptr), start_time);
    service_found = true;
  }

  RCLCPP_INFO_STREAM(
      Logger, "duration = " << duration << " service_found=" << service_found);
  CHECK(service_found);  // Test assertions - check that the servie was found
}

////////////////////////////////////////////////
// Test Case 2
////////////////////////////////////////////////

/* In this test case, the node under test (aka Auxiliary test node)
   is a topic talker, which got launched by the launcher.

   We will ceate a topic listener as part of the node performing the
   test (aka Integration test node).  And the test simply checks if
   the topic is recieved within the duration of the test. */

TEST_CASE_METHOD(MyTestsFixture, "test topic talker", "[topic]") {
  /**
   * 4.) Now, subscribe to a specific topic we're looking for:
   */
  bool got_topic = false;

  // Define a callback that captures the additional parameter
  struct ListenerCallback {
    explicit ListenerCallback(bool &gotTopic) : gotTopic_(gotTopic) {}
    void operator()(const String msg) const {
      RCLCPP_INFO_STREAM(Logger, "I heard:" << msg.data.c_str());
      gotTopic_ = true;
    }
    bool &gotTopic_;
  };

  auto subscriber = testerNode->create_subscription<String>(
      "topic", 10, ListenerCallback(got_topic));
  RCLCPP_INFO_STREAM(Logger, "Subscribed to 'topic'.");

  /**
   * 5.) Finally do the actual test:
   */
  std::time_t start_time = std::time(nullptr);
  double duration = 0;  // Duration in seconds
  const double timeout_seconds = TEST_DURATION;
  RCLCPP_INFO_STREAM(
      Logger, "duration = " << duration << " timeout=" << timeout_seconds);

  while (!got_topic && (duration < timeout_seconds)) {
    rclcpp::spin_some(testerNode);
    rclcpp::Rate(10.0).sleep();  // 10Hz
    duration = std::difftime(std::time(nullptr), start_time);
  }

  RCLCPP_INFO_STREAM(Logger,
                     "duration = " << duration << " got_topic=" << got_topic);
  CHECK(got_topic);  // Test assertions - check that the topic was received
}
