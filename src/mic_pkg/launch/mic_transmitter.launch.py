from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mic_pkg',
            executable='mic_transmitter_node',
            name='mic_transmitter',
            output='screen',
            parameters=[{
                'device': 'hw:2,0',
                'sample_rate': 16000,
                'channels': 1,
                'frames_per_period': 1024,
            }],
        ),
    ])
