from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_frame': 'camera_init',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',           # ← El que creamos arriba
                'mode': 'localization',
                'map_file_name': '/home/salvador/maps/mapa_prueba',
                'resolution': 0.05,
                'max_laser_range': 25.0,
                'minimum_travel_distance': 0.1,
                'minimum_travel_heading': 0.1,
                'height_filter_min': 0.15,
                'height_filter_max': 1.8,
            }]
        ),
    ]) 