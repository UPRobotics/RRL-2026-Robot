import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_pkg')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'ekbalam.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # Publishes the fixed TF: base_link → lidar_link (from URDF).
        # Update urdf/ekbalam.urdf once the real LiDAR mount offset is measured.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
        ),

        # Remaps FAST-LIO odometry frames to what Nav2 expects:
        #   /Odometry (camera_init → body)  →  /odom (odom → base_link)
        # Also broadcasts the odom → base_link TF.
        # Requires fastlio_pkg to be running separately (ros2 launch fastlio_pkg mapping.launch.py).
        Node(
            package='robot_pkg',
            executable='fast_lio_remapper_node',
            name='fast_lio_remapper',
            output='screen',
        ),

    ])
