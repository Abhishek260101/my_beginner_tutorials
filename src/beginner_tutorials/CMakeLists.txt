cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

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
)

# Create the subscriber executable
add_executable(listener src/simple_subscriber.cpp)
ament_target_dependencies(listener 
  rclcpp 
  std_msgs
)

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
)

ament_package()