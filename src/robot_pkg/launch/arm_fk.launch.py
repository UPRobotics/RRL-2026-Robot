from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Lanza el nodo de cinemática directa del brazo.

    Publica en /arm/end_effector_pose la posición y orientación del efector final.
    Requiere que:
    - robot_state_publisher esté publicando transformaciones TF2
    - El URDF esté disponible
    """
    return LaunchDescription([
        Node(
            package='robot_pkg',
            executable='arm_forward_kinematics.py',
            name='arm_forward_kinematics',
            output='screen',
        )
    ])
