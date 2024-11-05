#include "simple_subscriber.hpp"

SimpleSubscriber::SimpleSubscriber()
: Node("simple_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10,
        std::bind(&SimpleSubscriber::topic_callback, this,
        std::placeholders::_1));
}

void SimpleSubscriber::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSubscriber>());
    rclcpp::shutdown();
    return 0;
}