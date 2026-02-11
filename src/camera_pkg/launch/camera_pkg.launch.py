"""Launch the camera_pkg node with config directory parameter."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('camera_pkg')
    config_dir = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        Node(
            package='camera_pkg',
            executable='camera_pkg_node',
            name='camera_pkg',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'config_dir': config_dir,
            }],
        ),
    ])
