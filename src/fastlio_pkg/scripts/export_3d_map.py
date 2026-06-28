#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from datetime import datetime
import os
from pathlib import Path

class PLYExporter(Node):
    def __init__(self):
        super().__init__('ply_exporter')
        
        # Parámetros
        self.declare_parameter('team_name', 'RoboCup2026')
        self.declare_parameter('mission_name', 'Prelim1')
        self.declare_parameter('output_dir', '/home/salvador/robocup2026/maps')
        
        self.team_name = self.get_parameter('team_name').value
        self.mission_name = self.get_parameter('mission_name').value
        self.output_dir = self.get_parameter('output_dir').value
        
        # Variables
        self.points = []
        self.rgb_values = []
        self.temp_values = []
        self.all_points_received = False
        self.start_time = None
        
        # Suscripción
        self.subscription = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.cloud_callback,
            10)
        
        self.get_logger().info(f'PLY Exporter iniciado para {self.team_name} - {self.mission_name}')
        self.get_logger().info('Acumula puntos cloud. Cuando termines, ejecuta: ros2 service call /ply_exporter/save_map std_srvs/srv/Empty')

    def cloud_callback(self, msg):
        """Acumula puntos del cloud"""
        try:
            # Extrae puntos
            points_list = pc2.read_points(
                msg,
                field_names=["x", "y", "z"],
                skip_nans=True
            )
            
            for point in points_list:
                self.points.append([point[0], point[1], point[2]])
            
            if not self.start_time:
                self.start_time = datetime.now()
                self.get_logger().info(f'Recibiendo cloud... {len(self.points)} puntos acumulados')
            else:
                if len(self.points) % 10000 == 0:
                    self.get_logger().info(f'Acumulados: {len(self.points)} puntos')
                    
        except Exception as e:
            self.get_logger().error(f'Error en callback: {e}')

    def save_ply(self, filename=None):
        """Guarda el cloud acumulado en PLY"""
        if not self.points:
            self.get_logger().error('No hay puntos para guardar')
            return
        
        points_array = np.array(self.points, dtype=np.float32)
        
        if filename is None:
            timestamp = datetime.now().strftime('%H-%M-%S')
            filename = f'{self.team_name}-{self.mission_name}-{timestamp}-map.ply'
        
        filepath = os.path.join(self.output_dir, filename)
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Escribe PLY
        self.write_ply(filepath, points_array)
        self.get_logger().info(f'✓ PLY guardado: {filepath}')
        self.get_logger().info(f'  Total de puntos: {len(points_array)}')
        self.get_logger().info(f'  Bounds X: [{points_array[:,0].min():.2f}, {points_array[:,0].max():.2f}]')
        self.get_logger().info(f'  Bounds Y: [{points_array[:,1].min():.2f}, {points_array[:,1].max():.2f}]')
        self.get_logger().info(f'  Bounds Z: [{points_array[:,2].min():.2f}, {points_array[:,2].max():.2f}]')

    @staticmethod
    def write_ply(filename, points):
        """Escribe archivo PLY con formato correcto RoboCup"""
        with open(filename, 'w') as f:
            # Header
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write('comment RoboCup Rescue 2026\n')
            f.write(f'element vertex {len(points)}\n')
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')
            f.write('end_header\n')
            
            # Data
            for point in points:
                f.write(f'{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n')

def main(args=None):
    rclpy.init(args=args)
    node = PLYExporter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Guardando mapa antes de salir...')
        node.save_ply()
        node.get_logger().info('✓ Mapa guardado. Saliendo.')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()