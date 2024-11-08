/**
 * @file simple_publisher.hpp
 * @author Abhishek Avhad
 * @brief Header file for ROS2 publisher node implementation
 * @version 0.1
 * @date 2024-02-08
 *
 * @copyright Copyright (c) 2024 Abhishek Avhad
 *
 * @details This header defines a ROS2 publisher node that publishes string
 * messages with configurable frequency and provides a service to modify the
 * message
 */

#ifndef BEGINNER_TUTORIALS_SIMPLE_PUBLISHER_HPP_
#define BEGINNER_TUTORIALS_SIMPLE_PUBLISHER_HPP_

#include <example_interfaces/srv/set_bool.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

/**
 * @brief A ROS2 publisher node class
 *
 * @details SimplePublisher creates a ROS2 node that publishes messages to the
 *          /chatter topic at a configurable frequency. It also provides a
 * service to modify the base message and handles parameter updates.
 */
class SimplePublisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for SimplePublisher
   *
   * @param options Node configuration options
   * @details Initializes the publisher, service, and parameter handling
   */
  explicit SimplePublisher(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  /**
   * @brief Timer callback for periodic message publishing
   *
   * @details Called at regular intervals determined by publish_frequency_
   *          to publish messages to the /chatter topic
   */
  void timer_callback();

  /**
   * @brief Service callback to modify the base message
   *
   * @param request Service request containing boolean flag
   * @param response Service response with operation result
   * @details Changes base message between "Hello" and "Goodbye"
   */
  void change_string_callback(
      const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
      std::shared_ptr<example_interfaces::srv::SetBool::Response> response);

  /**
   * @brief Parameter update callback
   *
   * @param parameters Vector of parameters to update
   * @return Result indicating success or failure
   * @details Handles updates to publish_frequency parameter
   */
  rcl_interfaces::msg::SetParametersResult parameters_callback(
      const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Updates the publishing timer period
   *
   * @details Reconfigures the timer based on publish_frequency_
   */
  void update_timer_period();

  // Node components
  /** @brief Timer for periodic publishing */
  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  /** @brief Publisher for string messages */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_{nullptr};

  /** @brief Service for modifying base message */
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_{
      nullptr};

  /** @brief Handler for parameter callbacks */
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_{nullptr};

  // State variables
  /** @brief Counter for published messages */
  size_t count_{0};

  /** @brief Base message to be published */
  std::string base_message_{"Hello"};

  /** @brief Frequency of message publishing in Hz */
  double publish_frequency_{2.0};
};

#endif  // BEGINNER_TUTORIALS_SIMPLE_PUBLISHER_HPP_
