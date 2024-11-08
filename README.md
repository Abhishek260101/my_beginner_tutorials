# This packages runs simple custom talker and listener with service to change the string during runtime.

## Dependencies: ROS Humble, C++17, Bash
## Package deps: example_interfaces

## To run the package follow the steps below

### 1) clone the repository in your workspace
### 2) colcon build your workspace
### 3) Make sure to use "source install/setup.bash" before running the nodes
### 4) to run the node use "ros2 launch beginner_tutorials tutorial_launch.py"  
### 5) to change the string use "ros2 service call /change_string example_interfaces/srv/SetBool "data: true"
### 6) to change the frequency use "ros2 param set /talker publish_frequency 5.0"
