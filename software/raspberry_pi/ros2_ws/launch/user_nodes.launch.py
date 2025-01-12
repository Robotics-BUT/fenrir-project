from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='i2c_handler',
            executable='i2c_handler',
            name='i2c_handler',
            output='screen',
            respawn=True
        ),
        Node(
            package='buttons_handler',
            executable='buttons_handler',
            name='buttons_handler',
            output='screen',
            respawn=True
        ),
        Node(
            package='camera_handler_cpp',
            executable='camera_handler_cpp',
            name='camera_handler_cpp',
            output='screen',
            respawn=True
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type': 'serial',
                 'serial_port': '/dev/ttyUSB0',
                 'serial_baudrate': 115200,
                 'frame_id': 'lidar',
                 'inverted': False,
                 'angle_compensate': True}],
            output='screen',
            respawn=True,
            remappings=[
                ('/scan', '/bpc_prp_robot/lidar')
            ]
        ),
    ])
