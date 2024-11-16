# ROS2 Beginner Tutorials

Note: This repository is for ENPM700 course where focus is on following industry standard software developement practices.
This package implements a custom publisher-subscriber system with additional features like runtime string modification service, dynamic frequency adjustment, and TF2 frame broadcasting. 

## Overview
- Custom talker (publisher) and listener (subscriber)
- Service to change string content during runtime
- Dynamic frequency adjustment via ROS parameters
- TF2 frame broadcasting
- Integration testing

## Dependencies
- ROS2 Humble
- C++17
- example_interfaces
- tf2_ros
- catch_ros2 (for testing)

## Build Instructions

1. Clone the repository in your ROS2 workspace
```bash
cd ~/your_ros2_workspace/src
git clone <repository-url>
```

2. Build the package
```bash
cd ~/your_ros2_workspace
colcon build --packages-select beginner_tutorials
```

3. Source the setup file
```bash
source install/setup.bash
```

## Usage

### Launch the Nodes
```bash
# Using launch file (recommended)
ros2 launch beginner_tutorials tutorial_launch.py

# Or run publisher directly
ros2 run beginner_tutorials talker
```

### Runtime Modifications

#### Change Published String
```bash
ros2 service call /change_string example_interfaces/srv/SetBool "data: true"
```

#### Modify Publishing Frequency
```bash
ros2 param set /talker publish_frequency 5.0
```

### TF2 Features

#### View Transform Data
```bash
ros2 run tf2_ros tf2_echo world talk
```

#### Generate TF Tree Visualization
```bash
ros2 run tf2_tools view_frames
```

## Testing

### Method 1: Using Launch File
Terminal 1:
```bash
ros2 run beginner_tutorials talker
```

Terminal 2:
```bash
ros2 run beginner_tutorials integration_test_node
```

### Method 2: Using Colcon Test
```bash
# Clean build
rm -rf build/ install/ log/

# Build with tests
colcon build --packages-select beginner_tutorials

# Run tests
colcon test --packages-select beginner_tutorials

# View test results
colcon test-result --verbose
```

### Bag Recording and Playback
```bash
# Record all topics
ros2 launch beginner_tutorials bag.launch.py

# Play recorded bag (replace timestamp)
ros2 bag play ~/ros2_bag_files/recording_YYYY_MM_DD_HH_MM_SS
```

## License
[Include your license information here]

## Author
[Your Name]