#!/usr/bin/env python3
"""
Calcula la cinemática directa del brazo usando TF2.
Lee las transformaciones que publica robot_state_publisher.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import math


class ArmForwardKinematics(Node):
    def __init__(self):
        super().__init__('arm_forward_kinematics')

        # Buffer y listener para TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers para publicar la pose del efector final
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/arm/end_effector_pose',
            10
        )

        # Timer para consultar transformaciones cada 50ms (20Hz)
        self.timer = self.create_timer(0.05, self.compute_fk)

        self.get_logger().info("Forward Kinematics node iniciado")
        self.get_logger().info("Consultando transformaciones desde base_link...")

    def compute_fk(self):
        """Calcula FK leyendo transformaciones de TF2"""
        try:
            # Obtener transformación desde base_link hasta el efector final (claw tip)
            transform = self.tf_buffer.lookup_transform(
                "base_link",          # Frame de referencia
                "claw_tip",           # Efector final (punta del claw)
                rclpy.time.Time()
            )

            # Extraer posición
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            # Extraer orientación (quaternion)
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            # Crear mensaje PoseStamped
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "base_link"

            # Llenar posición
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z

            # Llenar orientación
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            # Publicar
            self.pose_pub.publish(pose)

            # Log cada segundo (debug)
            # self.get_logger().info(
            #     f"Efector final: ({x:.3f}, {y:.3f}, {z:.3f})"
            # )

        except Exception as e:
            # No mostrar error cada vez que falla (puede ser normal al inicio)
            pass


def main():
    rclpy.init()
    node = ArmForwardKinematics()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
