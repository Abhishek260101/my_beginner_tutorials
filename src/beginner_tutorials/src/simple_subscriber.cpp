#include "simple_subscriber.hpp"

// Constructor for the SimpleSubscriber class
SimpleSubscriber::SimpleSubscriber() : Node("simple_subscriber") {
  // Creating a subscription for std_msgs::msg::String with a queue size of 10
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10,
      // Binding the topic_callback function to the SimpleSubscriber instance
      std::bind(&SimpleSubscriber::topic_callback, this,
                std::placeholders::_1));
}

// Callback function for the subscription
void SimpleSubscriber::topic_callback(
    const std_msgs::msg::String::SharedPtr msg) {
  // Logging the received message data
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

// Main function
int main(int argc, char* argv[]) {
  // Initializing the ROS 2 system
  rclcpp::init(argc, argv);
  // Creating a SimpleSubscriber instance and spinning it to handle incoming messages
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  // Shutdown the ROS 2 system
  rclcpp::shutdown();
  // Return 0 to indicate successful execution
  return 0;
}
