from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='telemetry_ui_pkg',
            executable='telemetry_ui_node',
            name='telemetry_ui',
            output='screen',
            emulate_tty=True,
        ),
    ])
