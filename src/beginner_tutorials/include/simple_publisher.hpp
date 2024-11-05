#ifndef SIMPLE_PUBLISHER_HPP_
#define SIMPLE_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Define the SimplePublisher class, which inherits from rclcpp::Node
class SimplePublisher : public rclcpp::Node {
 public:
  // Constructor for the SimplePublisher class
  SimplePublisher();

 private:
  // Callback function for the timer
  void timer_callback();
  // Shared pointer to a timer
  rclcpp::TimerBase::SharedPtr timer_;
  // Shared pointer to a publisher for std_msgs::msg::String
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  // Counter to keep track of the number of messages published
  size_t count_;
};

#endif  // SIMPLE_PUBLISHER_HPP_
