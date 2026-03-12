import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    num_motors_arg = DeclareLaunchArgument(
        'num_motors',
        default_value='10',
        description='Number of motors to simulate (1-10)'
    )

    motor_sim = Node(
        package='robot_pkg',
        executable='motor_sim_node',
        name='motor_sim_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'num_motors': LaunchConfiguration('num_motors'),
        }],
    )

    # Also launch telemetry_node to aggregate and relay
    telemetry_node = Node(
        package='telemetry_pkg',
        executable='telemetry_node',
        name='telemetry_node',
        output='screen',
        emulate_tty=True,
    )

    # Launch the telemetry UI
    telemetry_ui_pkg_dir = get_package_share_directory('telemetry_ui_pkg')
    telemetry_ui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(telemetry_ui_pkg_dir, 'launch', 'telemetry_ui.launch.py')
        )
    )

    return LaunchDescription([
        num_motors_arg,
        motor_sim,
        telemetry_node,
        telemetry_ui,
    ])
