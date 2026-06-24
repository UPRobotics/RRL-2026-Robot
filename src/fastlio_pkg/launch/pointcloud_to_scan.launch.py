from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # odom → camera_init (identidad, para que slam_toolbox vea odom→base_link)
        # body → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_link']
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/home/salvador/robocup2026/config/ekf.yaml'],
            remappings=[('odometry/filtered', '/odom')]
        ),
        # PointCloud → LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'target_frame': 'base_link',
                'min_height': 0.15,
                'max_height': 1.8,
                'angle_min': -3.14159,
                'angle_max':  3.14159,
                'angle_increment': 0.0087,
                'range_min': 0.2,
                'range_max': 25.0,
            }],
            remappings=[
                ('cloud_in', '/cloud_registered'),
                ('scan', '/scan')
            ]
        )
    ])