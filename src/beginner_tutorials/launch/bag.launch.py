from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from datetime import datetime
import os

def generate_launch_description():
    # Declare launch arguments
    record_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='True',
        description='Record bag file if True'
    )

    # Get the current timestamp for bag file naming
    timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    
    # Create bag directory in home folder if it doesn't exist
    bag_directory = os.path.expanduser('~/ros2_bag_files')
    os.makedirs(bag_directory, exist_ok=True)
    
    # Define bag filepath
    bag_filepath = os.path.join(bag_directory, f'recording_{timestamp}')

    # Start the talker node
    talker_node = Node(
        package='beginner_tutorials',
        executable='talker',
        name='publisher',
        output='screen'
    )

    # Configure bag recording with all topics
    bag_record = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record_bag')),
        cmd=['ros2', 'bag', 'record', '-a', '-o', bag_filepath],
        shell=True,
        output='screen'
    )

    return LaunchDescription([
        record_arg,
        talker_node,
        bag_record
    ])