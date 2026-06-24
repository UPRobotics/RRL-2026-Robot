import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # ==================== Rutas del paquete ====================
    package_path = get_package_share_directory('fastlio_pkg')
    
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio.rviz')

    # ==================== Argumentos ====================
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', 
        default_value=default_config_path,
        description='Path to yaml config directory'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', 
        default_value='mid360.yaml',        # Cambia si usas avia.yaml u otro
        description='Config file name'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', 
        default_value='false',              # Por defecto desactivado (porque usamos Foxglove)
        description='Launch RViz2'
    )

    declare_foxglove_cmd = DeclareLaunchArgument(
        'foxglove', 
        default_value='true',               # Por defecto activado
        description='Launch Foxglove Bridge'
    )

    # ==================== Nodos ====================

    # Cargar URDF del robot
    robot_description_path = get_package_share_directory('ensamble_flaca_description')
    robot_urdf_file = os.path.join(robot_description_path, 'urdf', 'ensamble_flaca.xacro')

    robot_description = Command(['xacro ', robot_urdf_file])

    # Nodo principal de FAST_LIO
    fast_lio_node = Node(
        package='fastlio_pkg',
        executable='fastlio_mapping',
        parameters=[
            PathJoinSubstitution([LaunchConfiguration('config_path'), LaunchConfiguration('config_file')]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Robot State Publisher - publica los transforms del URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # Nodo corrected_odom - publica TF odom → base_link
    corrected_odom_node = Node(
        package='fastlio_pkg',
        executable='corrected_odom.py',
        output='screen'
    )

    # Nodo auto_level - publica TF map → camera_init (nivelación basada en IMU)
    auto_level_node = Node(
        package='fastlio_pkg',
        executable='auto_level.py',
        output='screen'
    )

    # Nodo RViz2 (solo se lanza si rviz:=true)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    # Nodo Foxglove Bridge (solo se lanza si foxglove:=true)
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'topic_whitelist': ['/.*'],     #Publica todos los topicos
            'use_compression': True
        }],
        condition=IfCondition(LaunchConfiguration('foxglove'))
    )

    # ==================== Launch Description ====================
    ld = LaunchDescription()

    # Agregar argumentos
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_foxglove_cmd)

    # Agregar nodos
    ld.add_action(fast_lio_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(corrected_odom_node)
    ld.add_action(auto_level_node)
    ld.add_action(rviz_node)
    ld.add_action(foxglove_bridge_node)

    return ld
