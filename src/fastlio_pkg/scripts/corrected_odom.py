#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import math

class CorrectedOdom(Node):
    def __init__(self):
        super().__init__('corrected_odom')
        self.correction_q = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry, '/Odometry', self.callback, 10)
        self.pub = self.create_publisher(
            Odometry, '/odom_corrected', 10)
        self.get_logger().info('Esperando TF map → camera_init...')

    def get_correction(self):
        if self.correction_q is not None:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'camera_init', rclpy.time.Time())
            q = tf.transform.rotation
            self.correction_q = np.array([q.x, q.y, q.z, q.w])
            self.get_logger().info(
                f'Corrección obtenida: '
                f'qx={q.x:.4f} qy={q.y:.4f} qz={q.z:.4f} qw={q.w:.4f}')
            return True
        except Exception:
            return False

    def quat_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return np.array([
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ])

    def rotate_vector(self, q, v):
        qv = np.array([v[0], v[1], v[2], 0.0])
        q_conj = np.array([-q[0], -q[1], -q[2], q[3]])
        tmp = self.quat_multiply(q, qv)
        result = self.quat_multiply(tmp, q_conj)
        return result[:3]

    def callback(self, msg):
        if not self.get_correction():
            return

        # Corrige la posición
        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        pos_corrected = self.rotate_vector(self.correction_q, pos)

        # Corrige la orientación completa
        q_orig = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        q_corrected = self.quat_multiply(self.correction_q, q_orig)

        # Extrae solo el yaw — nav2 es 2D, no necesita pitch ni roll
        yaw = math.atan2(
            2.0 * (q_corrected[3]*q_corrected[2] + q_corrected[0]*q_corrected[1]),
            1.0 - 2.0 * (q_corrected[1]*q_corrected[1] + q_corrected[2]*q_corrected[2])
        )

        # Publica odometría corregida en 2D
        odom_out = Odometry()
        odom_out.header.stamp = msg.header.stamp
        odom_out.header.frame_id = 'map'
        odom_out.child_frame_id = 'base_link'
        odom_out.pose.pose.position.x = float(pos_corrected[0])
        odom_out.pose.pose.position.y = float(pos_corrected[1])
        odom_out.pose.pose.position.z = 0.0
        odom_out.pose.pose.orientation.x = 0.0
        odom_out.pose.pose.orientation.y = 0.0
        odom_out.pose.pose.orientation.z = float(math.sin(yaw/2))
        odom_out.pose.pose.orientation.w = float(math.cos(yaw/2))
        odom_out.twist = msg.twist
        self.pub.publish(odom_out)

        # Publica TF body → base_link solo con yaw
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(pos_corrected[0])
        t.transform.translation.y = float(pos_corrected[1])
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = float(math.sin(yaw/2))
        t.transform.rotation.w = float(math.cos(yaw/2))
        self.br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(CorrectedOdom())

if __name__ == '__main__':
    main()
