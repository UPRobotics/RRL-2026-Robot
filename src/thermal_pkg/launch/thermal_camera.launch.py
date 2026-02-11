from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the thermal camera'
    )

    thermal_camera_node = Node(
        package='thermal_pkg',
        executable='thermal_camera_node',
        name='thermal_camera_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 115200,
            'publish_rate': 4.0,
        }]
    )

    thermal_display_node = Node(
        package='thermal_pkg',
        executable='thermal_display_node',
        name='thermal_display_node',
        output='screen',
    )

    return LaunchDescription([
        serial_port_arg,
        thermal_camera_node,
        thermal_display_node,
    ])
