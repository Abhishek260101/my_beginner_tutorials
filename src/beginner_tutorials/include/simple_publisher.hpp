// include/simple_publisher.hpp
// Copyright 2024 Abhishek Avhad
#ifndef BEGINNER_TUTORIALS_SIMPLE_PUBLISHER_HPP_
#define BEGINNER_TUTORIALS_SIMPLE_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <example_interfaces/srv/set_bool.hpp>
#include <memory>
#include <string>
#include <vector>

class SimplePublisher : public rclcpp::Node {
 public:
    explicit SimplePublisher(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
    void timer_callback();
    void change_string_callback(
        const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
        std::shared_ptr<example_interfaces::srv::SetBool::Response> response);

    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter>& parameters);

    void update_timer_period();

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_{nullptr};
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_{nullptr};
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_{nullptr};
    size_t count_{0};
    std::string base_message_{"Hello"};
    double publish_frequency_{2.0};
};

#endif  // BEGINNER_TUTORIALS_SIMPLE_PUBLISHER_HPP_
