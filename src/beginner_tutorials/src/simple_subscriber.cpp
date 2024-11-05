// src/simple_subscriber.cpp
// Copyright 2024 Abhishek Avhad
#include "simple_subscriber.hpp"

SimpleSubscriber::SimpleSubscriber() : Node("simple_subscriber") {
  RCLCPP_INFO_STREAM(this->get_logger(), "Initializing Subscriber Node");
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/chatter", 10,
      std::bind(&SimpleSubscriber::topic_callback, this,
                std::placeholders::_1));

  if (!subscription_) {
    RCLCPP_FATAL(this->get_logger(), "Failed to create subscriber");
    return;
  }

  RCLCPP_DEBUG_STREAM(this->get_logger(), "Subscriber created successfully");
}

void SimpleSubscriber::topic_callback(
    const std_msgs::msg::String::SharedPtr msg) {
  if (msg->data.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Received empty message");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  rclcpp::shutdown();
  return 0;
}
