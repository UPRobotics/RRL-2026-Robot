#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np

class AutoLevel(Node):
    def __init__(self):
        super().__init__('auto_level')
        self.corrected = False
        self.samples = []
        self.num_samples = 50  # promedia 50 muestras del IMU
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.sub = self.create_subscription(
            Imu, '/livox/imu', self.callback, 10)
        self.get_logger().info(
            'Leyendo IMU para calcular inclinación real...')

    def callback(self, msg):
        if self.corrected:
            return

        # Acumula muestras de aceleración
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        self.samples.append([ax, ay, az])

        if len(self.samples) < self.num_samples:
            return

        # Promedia las muestras para reducir ruido
        avg = np.mean(self.samples, axis=0)
        ax, ay, az = avg

        self.get_logger().info(
            f'Aceleración promedio: ax={ax:.4f} ay={ay:.4f} az={az:.4f}')

        # Calcula la inclinación desde la gravedad
        # La gravedad apunta en -Z del mundo
        # Si el sensor está nivelado: ax=0, ay=0, az=-9.81
        # Si está inclinado, ax y ay tienen componentes

        g = np.array([ax, ay, az])
        g_norm = g / np.linalg.norm(g)

        # Vector de gravedad en mundo horizontal
        g_world = np.array([0.0, 0.0, 1.0])

        # Calcula quaternion que rota g_norm a g_world
        # usando la fórmula de rotación entre dos vectores
        cross = np.cross(g_norm, g_world)
        dot = np.dot(g_norm, g_world)

        cross_norm = np.linalg.norm(cross)
        if cross_norm < 1e-6:
            # Ya está nivelado
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            cross_normalized = cross / cross_norm
            angle = np.arctan2(cross_norm, dot)
            qx = float(cross_normalized[0] * np.sin(angle/2))
            qy = float(cross_normalized[1] * np.sin(angle/2))
            qz = float(cross_normalized[2] * np.sin(angle/2))
            qw = float(np.cos(angle/2))

        self.get_logger().info(
            f'Corrección calculada: '
            f'qx={qx:.4f} qy={qy:.4f} qz={qz:.4f} qw={qw:.4f}')

        # Publica TF: camera_init_level → camera_init
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'camera_init'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.broadcaster.sendTransform(t)
        self.corrected = True
        self.get_logger().info(
            'camera_init_level es ahora horizontal '
            'basado en la gravedad real del IMU.')

def main():
    rclpy.init()
    rclpy.spin(AutoLevel())

if __name__ == '__main__':
    main()