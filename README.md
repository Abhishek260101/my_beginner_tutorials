# my_beginner_tutorials

ros2 launch beginner_tutorials tutorial_launch.py
ros2 service call /change_string example_interfaces/srv/SetBool "data: true"
ros2 param set /talker publish_frequency 5.0


cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $(find src/beginner_tutorials -name *.cpp -o -name *.hpp)
