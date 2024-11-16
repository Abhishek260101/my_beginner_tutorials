cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# Include directories
include_directories(include)

# Create the publisher executable
add_executable(talker 
  src/simple_publisher.cpp
)

ament_target_dependencies(talker 
  rclcpp 
  std_msgs
  example_interfaces
  tf2
  tf2_ros
  geometry_msgs
)

# Creating the subscriber executable
add_executable(listener 
  src/simple_subscriber.cpp
)

ament_target_dependencies(listener 
  rclcpp 
  std_msgs
)

# Testing configuration
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(catch_ros2 REQUIRED)

  # Create test executable
  add_executable(integration_test_node
    test/integration_test.cpp
  )

  target_include_directories(integration_test_node PUBLIC
    ${catch_ros2_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(integration_test_node
    ${catch_ros2_LIBRARIES}
  )

  ament_target_dependencies(integration_test_node
    rclcpp
    std_msgs
    tf2_msgs
    tf2
    tf2_ros
    geometry_msgs
    example_interfaces
    catch_ros2
  )

  # Register the test
  add_test(
    NAME integration_test
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/integration_test_node
  )

  # Install the test executable
  install(TARGETS
    integration_test_node
    DESTINATION lib/${PROJECT_NAME}
  )

  # Install the test launch file
  install(FILES
    launch/test_service.launch.yaml
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

# Install executables
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY 
  include/
  DESTINATION include/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export dependencies
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  example_interfaces
  tf2
  tf2_ros
  geometry_msgs
)

ament_package()