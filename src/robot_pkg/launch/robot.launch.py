import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node # Importante para declarar nodos

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_pkg')
    telemetry_pkg_dir = get_package_share_directory('telemetry_pkg')

    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'arm.launch.py')
        )
    )

    body = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'body.launch.py')
        )
    )

    telemetry = Node(
            package='telemetry_pkg',
            executable='telemetry_node',
            name='telemetry_processor',
            output='screen'
        )

    return LaunchDescription([arm, body, telemetry])
