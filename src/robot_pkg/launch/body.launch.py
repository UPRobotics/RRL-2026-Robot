from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_pkg',
            executable='body_node',
            name='body_node',
            output='screen',
            parameters=[{
                'left_id':                0,
                'left_baudrate':          115200,
                'left_timeout':           1000,
                'right_id':               1,
                'right_baudrate':         115200,
                'right_timeout':          1000,
                'left_flipper_id':        2,
                'left_flipper_baudrate':  115200,
                'left_flipper_timeout':   1000,
                'right_flipper_id':       3,
                'right_flipper_baudrate': 115200,
                'right_flipper_timeout':  1000,
            }],
        )
    ])
