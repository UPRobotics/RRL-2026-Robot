from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='detections_pkg',
            executable='detections_pkg_node',
            name='detections_pkg',
            output='screen',
        ),
        # Add more nodes here as needed, e.g.:
        # Node(
        #     package='magnetometer_pkg',
        #     executable='magnetometer_sender',
        #     name='magnetometer_sender',
        #     output='screen',
        # ),
    ])
