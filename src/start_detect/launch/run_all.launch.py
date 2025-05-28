from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='show_result',
            executable='show_result',
            name='show_result_node',
            output='screen'
        ),
        Node(
            package='start_detect',
            executable='start_detect',
            name='start_detect_node',
            output='screen'
        ),
    ])