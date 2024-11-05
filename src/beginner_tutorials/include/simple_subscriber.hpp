// include/beginner_tutorials/simple_subscriber.hpp
// Copyright 2024 Abhishek Avhad
#ifndef MY_BEGINNER_TUTORIALS_SRC_BEGINNER_TUTORIALS_INCLUDE_SIMPLE_SUBSCRIBER_HPP_
#define MY_BEGINNER_TUTORIALS_SRC_BEGINNER_TUTORIALS_INCLUDE_SIMPLE_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

class SimpleSubscriber : public rclcpp::Node {
 public:
    SimpleSubscriber();

 private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif  // MY_BEGINNER_TUTORIALS_SRC_BEGINNER_TUTORIALS_INCLUDE_SIMPLE_SUBSCRIBER_HPP_
