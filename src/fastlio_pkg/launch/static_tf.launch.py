from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # body → base_link

        # Nodo de corrección automática de inclinación
        # Lee el primer /Odometry y publica camera_init_level horizontal
        Node(
            package='fastlio_pkg',
            executable='auto_level.py',
            name='auto_level',
            output='screen'
        ),
        # Nodo que corrige la odometría de Fast-LIO usando el TF de corrección
        Node(
            package='fastlio_pkg',
            executable='corrected_odom.py',
            name='corrected_odom',
            output='screen'
        ),
        Node(
            package='fastlio_pkg',
            executable='filter_cloud.py',
            name='filter_cloud',
            output='screen'
        ),
        # odom conectado al frame nivelado para nav2
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='level_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1',
                      'map', 'odom'],
            output='screen'
        ),
    ])