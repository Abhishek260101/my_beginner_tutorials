#include "simple_publisher.hpp"
#include <chrono>

using namespace std::chrono_literals;

SimplePublisher::SimplePublisher(const rclcpp::NodeOptions& options)
: Node("simple_publisher", options), count_(0) {
    // Declare parameters with descriptor for better documentation
    rcl_interfaces::msg::ParameterDescriptor freq_desc;
    freq_desc.description = "Publishing frequency in Hz";
    freq_desc.floating_point_range.resize(1);
    freq_desc.floating_point_range[0].from_value = 0.1;
    freq_desc.floating_point_range[0].to_value = 30.0;
    
    this->declare_parameter("publish_frequency", 2.0, freq_desc);
    publish_frequency_ = this->get_parameter("publish_frequency").as_double();
    
    RCLCPP_INFO_STREAM(this->get_logger(), 
        "Starting publisher with frequency: " << publish_frequency_ << " Hz");
    
    publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);
    
    // Set up parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&SimplePublisher::parameters_callback, this, std::placeholders::_1));
    
    // Create initial timer
    update_timer_period();

    service_ = this->create_service<example_interfaces::srv::SetBool>(
        "change_string",
        std::bind(&SimplePublisher::change_string_callback, this,
        std::placeholders::_1, std::placeholders::_2));
        
    RCLCPP_INFO(this->get_logger(), "Publisher initialized on /chatter topic");
}

void SimplePublisher::update_timer_period() {
    auto timer_period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / publish_frequency_));
    timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&SimplePublisher::timer_callback, this));
}

rcl_interfaces::msg::SetParametersResult SimplePublisher::parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto& param : parameters) {
        if (param.get_name() == "publish_frequency") {
            double new_freq = param.as_double();
            if (new_freq > 0.0) {
                publish_frequency_ = new_freq;
                RCLCPP_INFO(this->get_logger(), 
                    "Updating publishing frequency to %.1f Hz", publish_frequency_);
                update_timer_period();
                result.successful = true;
            } else {
                result.successful = false;
                result.reason = "Frequency must be positive";
            }
        }
    }
    return result;
}

void SimplePublisher::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = base_message_ + " World! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

void SimplePublisher::change_string_callback(
    const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
    std::shared_ptr<example_interfaces::srv::SetBool::Response> response) {
    if (request->data) {
        base_message_ = "Goodbye";
        response->success = true;
        response->message = "Changed base string to 'Goodbye'";
        RCLCPP_INFO_STREAM(this->get_logger(), "Base string changed to: " << base_message_);
    } else {
        base_message_ = "Hello";
        response->success = true;
        response->message = "Changed base string back to 'Hello'";
        RCLCPP_INFO_STREAM(this->get_logger(), "Base string changed to: " << base_message_);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}