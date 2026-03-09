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
                'sample_rate': 48000,
                'channels': 1,
                'frames_per_period': 512,
            }],
        ),
        Node(
            package='mic_pkg',
            executable='mic_receiver_node',
            name='mic_receiver',
            output='screen',
            parameters=[{
                'device': 'default',
                'sample_rate': 48000,
                'channels': 1,
                'frames_per_period': 512,
            }],
        ),
    ])
