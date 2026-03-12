from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_pkg',
            executable='arm_node',
            name='arm_node',
            output='screen',
            parameters=[{
                'hip_id':             4,
                'hip_baudrate':       115200,
                'hip_timeout':        1000,
                'shoulder_id':        5,
                'shoulder_baudrate':  115200,
                'shoulder_timeout':   1000,
                'elbow_id':           6,
                'elbow_baudrate':     115200,
                'elbow_timeout':      1000,
                'roll_id':            7,
                'roll_baudrate':      115200,
                'roll_timeout':       1000,
                'pitch_id':           8,
                'pitch_baudrate':     115200,
                'pitch_timeout':      1000,
                'claw_id':            9,
                'claw_baudrate':      115200,
                'claw_timeout':       1000,
            }],
        )
    ])
