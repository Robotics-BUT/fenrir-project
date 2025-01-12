from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rgb_leds_handler',
            executable='rgb_leds_handler',
            name='rgb_leds_handler',
            output='screen',
            respawn=True
        ),
    ])
