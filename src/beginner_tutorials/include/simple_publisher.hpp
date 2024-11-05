#ifndef SIMPLE_PUBLISHER_HPP_
#define SIMPLE_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SimplePublisher : public rclcpp::Node {
public:
    SimplePublisher();

private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

#endif  // SIMPLE_PUBLISHER_HPP_
