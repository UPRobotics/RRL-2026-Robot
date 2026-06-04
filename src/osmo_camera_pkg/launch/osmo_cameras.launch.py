import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('osmo_camera_pkg')
    config_dir = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        Node(
            package='osmo_camera_pkg',
            executable='osmo_stream_node',
            name='osmo_stream_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'config_dir': config_dir}],
        ),
        Node(
            package='osmo_camera_pkg',
            executable='osmo_display_node',
            name='osmo_display_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'config_dir': config_dir}],
        ),
    ])
