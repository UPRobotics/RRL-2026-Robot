#!/usr/bin/env python3
"""
Launch file para Autonomy Dashboard
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Parámetros
    config_dir = os.path.join(
        get_package_share_directory('autonomy_dashboard_pkg'),
        'config'
    )
    autonomy_params = os.path.join(config_dir, 'autonomy_params.yaml')

    team_name_arg = DeclareLaunchArgument(
        'team_name',
        default_value='RoboCup2026',
        description='Nombre del equipo'
    )

    mission_arg = DeclareLaunchArgument(
        'mission',
        default_value='Prelim1',
        description='Nombre de la misión'
    )

    return LaunchDescription([
        team_name_arg,
        mission_arg,

        # Nodo de Autonomy Dashboard
        Node(
            package='autonomy_dashboard_pkg',
            executable='autonomy_dashboard_node.py',
            name='autonomy_dashboard_node',
            output='screen',
            parameters=[
                autonomy_params,
                {
                    'team_name': LaunchConfiguration('team_name'),
                    'mission': LaunchConfiguration('mission'),
                }
            ]
        ),

        # Interfaz PyQt5 (opcional)
        Node(
            package='autonomy_dashboard_pkg',
            executable='autonomy_ui.py',
            name='autonomy_dashboard_ui',
            output='screen',
        ),
    ])
