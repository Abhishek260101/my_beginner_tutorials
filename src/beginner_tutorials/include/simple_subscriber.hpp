#ifndef SIMPLE_SUBSCRIBER_HPP_
#define SIMPLE_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Define the SimpleSubscriber class, which inherits from rclcpp::Node
class SimpleSubscriber : public rclcpp::Node {
 public:
  // Constructor for the SimpleSubscriber class
  SimpleSubscriber();

 private:
  // Callback function for the subscription
  void topic_callback(const std_msgs::msg::String::SharedPtr msg);

  // Shared pointer to a subscription for std_msgs::msg::String
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif  // SIMPLE_SUBSCRIBER_HPP_
