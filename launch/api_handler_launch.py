from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='api_handler',
            executable='api_handler_node',
            name='api_handler_node',
            output='screen',
            emulate_tty=True, # Required for seeing print statements in the console
        ),
    ])