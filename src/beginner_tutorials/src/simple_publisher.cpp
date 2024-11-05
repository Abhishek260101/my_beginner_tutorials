#include "simple_publisher.hpp"

// Constructor for the SimplePublisher class
SimplePublisher::SimplePublisher() : Node("simple_publisher"), count_(0) {
  // Creating a publisher for std_msgs::msg::String
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  // Creating a wall timer with a period of 500 milliseconds
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      // Bind the timer callback function to the SimplePublisher instance
      std::bind(&SimplePublisher::timer_callback, this));
}

// Callback function for the timer
void SimplePublisher::timer_callback() {
  // Creating a new message of type std_msgs::msg::String
  auto message = std_msgs::msg::String();
  // Set the message data to "Hello World! " followed by the current count
  message.data = "Hello World! " + std::to_string(count_++);
  // Log the message data
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  // Publish the message
  publisher_->publish(message);
}

// Main function
int main(int argc, char* argv[]) {
  // Initializing the ROS 2 system
  rclcpp::init(argc, argv);
  // Create a SimplePublisher instance and spin it
  rclcpp::spin(std::make_shared<SimplePublisher>());
  // Shutdown the ROS 2 system
  rclcpp::shutdown();
  // Return 0 to indicate successful execution
  return 0;
}
