#ifndef SIMPLE_PUBLISHER_HPP_
#define SIMPLE_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <example_interfaces/srv/set_bool.hpp>

class SimplePublisher : public rclcpp::Node {
public:
    SimplePublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void timer_callback();
    void change_string_callback(
        const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
        std::shared_ptr<example_interfaces::srv::SetBool::Response> response);
    
    // Add parameter callback
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter>& parameters);
    
    void update_timer_period();
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    size_t count_;
    std::string base_message_{"Hello"};
    double publish_frequency_;
};

#endif  // SIMPLE_PUBLISHER_HPP_
