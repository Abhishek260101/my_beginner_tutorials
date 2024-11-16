/**
 * @file integration_test.cpp
 * @brief Test suite for ROS2 publisher functionality
 * @version 0.1
 * @date 2024-02-08
 * @copyright Copyright (c) 2024 Abhishek Avhad
 */

#define CATCH_CONFIG_RUNNER
#include <catch_ros2/catch_ros2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using rclcpp::Node;
using std::string;
using std::chrono::seconds;
using std_msgs::msg::String;
using std::chrono_literals::operator""s;

/**
 * @class TestFixture
 * @brief A test fixture class for setting up and handling ROS2 subscribers.
 */
class TestFixture {
 public:
  /**
   * @brief Constructor for the TestFixture class. Initializes the test node and
   * subscriptions.
   */
  TestFixture() {
    // Create a shared pointer to a new node named "test_listener"
    test_node_ = std::make_shared<Node>("test_listener");
    message_received_ = false;
    tf_received_ = false;
    received_message_ = "";

    // Subscription to the "/chatter" topic for receiving string messages
    subscription_ = test_node_->create_subscription<String>(
        "/chatter", 10, [this](const String::SharedPtr msg) {
          message_received_ = true;
          received_message_ = msg->data;
          // Log the received message
          RCLCPP_INFO(test_node_->get_logger(), "Received message: '%s'",
                      msg->data.c_str());
        });

    // Subscription to the "/tf" topic for receiving transform messages
    tf_subscription_ =
        test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10, [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
              if (!msg->transforms.empty()) {
                tf_received_ = true;
                last_transform_ = msg->transforms[0];
                // Log the received transform data
                RCLCPP_INFO(test_node_->get_logger(), "Received TF: %s -> %s",
                            last_transform_.header.frame_id.c_str(),
                            last_transform_.child_frame_id.c_str());
              }
            });
  }

  /**
   * @brief Waits for messages to be received within a specified timeout.
   * @param timeout Duration to wait before timing out.
   * @return True if both messages were received, false if timeout occurred.
   */
  bool waitForMessages(seconds timeout = 5s) {
    auto start = std::chrono::steady_clock::now();
    rclcpp::Rate rate(10);  // Spin at 10 Hz

    // Loop until the message and TF data are received or timeout
    while (rclcpp::ok() && (!message_received_ || !tf_received_)) {
      rclcpp::spin_some(test_node_);  // Process any available messages
      rate.sleep();  // Sleep for a small duration to avoid busy waiting
      auto now = std::chrono::steady_clock::now();
      if (now - start > timeout) {
        // Log an error if the timeout is reached
        RCLCPP_ERROR(test_node_->get_logger(), "Timeout waiting for messages");
        return false;
      }
    }
    return message_received_ && tf_received_;
  }

  /**
   * @brief Getter for the last received message.
   * @return The last received message as a string.
   */
  string getLastMessage() const { return received_message_; }

  /**
   * @brief Getter for the last received transform message.
   * @return The last received TransformStamped message.
   */
  geometry_msgs::msg::TransformStamped getLastTransform() const {
    return last_transform_;
  }

 private:
  rclcpp::Node::SharedPtr test_node_;
  // Subscription for string messages
  rclcpp::Subscription<String>::SharedPtr
      subscription_;
  // Subscription for TF messages
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr
      tf_subscription_;
  bool message_received_;
  bool tf_received_;
  string received_message_;
  // Stores the last received TF message
  geometry_msgs::msg::TransformStamped
      last_transform_;
};

/**
 * @brief Test case for verifying publisher functionality.
 */
TEST_CASE("Test Publisher Node", "[publisher]") {
  RCLCPP_INFO(rclcpp::get_logger("test_case"), "Starting publisher test");

  TestFixture fixture;

  SECTION("Verify message and TF publishing") {
    // Ensure that messages are received within the default timeout
    REQUIRE(fixture.waitForMessages());

    // Get and verify the received message
    string received_message = fixture.getLastMessage();
    // Check if the message is not empty
    CHECK(!received_message.empty());
    CHECK(received_message.find("Hello") != string::npos);

    // Get and verify the last transform message
    auto transform = fixture.getLastTransform();
    CHECK(transform.header.frame_id == "world");
    CHECK(transform.child_frame_id == "talk");
    CHECK(transform.transform.translation.z == Catch::Approx(0.5).margin(0.01));

    RCLCPP_INFO(rclcpp::get_logger("test_case"), "Test completed successfully");
  }
}

/**
 * @brief Main function for running the ROS2 test.
 */
int main(int argc, char* argv[]) {
  // Initialize ROS2
  rclcpp::init(argc, argv);
  Catch::Session().run(argc, argv);
  rclcpp::shutdown();
  return 0;
}
