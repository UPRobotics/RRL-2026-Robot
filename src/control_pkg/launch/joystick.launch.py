from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Lanza el nodo de joystick para la estación terrestre.

    Requiere el paquete 'joy' de ROS 2:
        sudo apt install ros-humble-joy

    Publica (BEST_EFFORT, depth=1):
        /joystick/left_y   → orugas adelante/atrás
        /joystick/left_x   → orugas rotación (diferencial)
        /joystick/right_y  → flipper delantero
        /joystick/right_x  → flipper trasero
    """
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{
            "device_id": 0,
            "deadzone": 0.05,
            "autorepeat_rate": 50.0,   # 50 Hz → 20 ms entre mensajes
            "coalesce_interval_ms": 1,
        }],
    )

    joystick_node = Node(
        package="control_pkg",
        executable="joystick_node",
        name="joystick_node",
    )

    return LaunchDescription([joy_node, joystick_node])
