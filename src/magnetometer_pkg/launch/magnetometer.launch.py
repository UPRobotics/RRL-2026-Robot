from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='magnetometer_pkg',
            executable='magnetometer_sender',
            name='magnetometer_sender',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baudrate': 115200,
                'timeout': 1.0,
            }],
        ),
        Node(
            package='magnetometer_pkg',
            executable='magnetometer_receiver',
            name='magnetometer_receiver',
            output='screen',
        ),
    ])
