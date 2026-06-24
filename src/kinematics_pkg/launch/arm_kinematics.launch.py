from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the arm kinematics node.

    Publishes:
    - /arm/end_effector_pose (PoseStamped) - Current end effector position
    - /arm/inverse_kinematics_solution (JointState) - IK solution

    Subscribes to:
    - /joint_states (sensor_msgs/JointState) - Current joint angles

    Parameters:
    - arm_base_frame: Base reference frame (default: "base_link")
    - arm_end_effector_frame: End effector frame (default: "claw_tip")
      Note: FK calculation includes the claw_tip offset from URDF joint roll_to_claw

    Requires:
    - robot_state_publisher running to publish TF2
    - /joint_states being published (from arm_state_publisher or similar)
    """
    return LaunchDescription([
        Node(
            package='kinematics_pkg',
            executable='arm_kinematics_node',
            name='arm_kinematics',
            output='screen',
            parameters=[
                {'arm_base_frame': 'base_link'},
                {'arm_end_effector_frame': 'claw_tip'},
            ],
        )
    ])
