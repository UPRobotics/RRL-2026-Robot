#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class FilterCloud(Node):
    def __init__(self):
        super().__init__('filter_cloud')

        # Distancia mínima — elimina el cuerpo del robot
        # Ajusta este valor según el tamaño de tu robot
        self.range_min = 0.6   # puntos a menos de 50cm = robot
        self.range_max = 10.0  # máximo del sensor

        self.sub = self.create_subscription(
            PointCloud2, '/cloud_registered', self.callback, 10)
        self.pub = self.create_publisher(
            PointCloud2, '/cloud_filtered', 10)

        self.get_logger().info(
            f'Filtrando puntos: '
            f'min={self.range_min}m max={self.range_max}m')

    def callback(self, msg):
        points_list = list(pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))

        if not points_list:
            return

        pts = np.array([[p[0], p[1], p[2]] for p in points_list],
                       dtype=np.float32)

        if pts.ndim != 2 or pts.shape[1] != 3:
            return

        # Calcula distancia 3D de cada punto al sensor
        distances = np.sqrt(pts[:,0]**2 + pts[:,1]**2 + pts[:,2]**2)

        # Filtra por distancia
        mask = (distances >= self.range_min) & (distances <= self.range_max)
        filtered = pts[mask]

        if len(filtered) == 0:
            return

        new_msg = pc2.create_cloud_xyz32(msg.header, filtered.tolist())
        self.pub.publish(new_msg)

def main():
    rclpy.init()
    rclpy.spin(FilterCloud())

if __name__ == '__main__':
    main()