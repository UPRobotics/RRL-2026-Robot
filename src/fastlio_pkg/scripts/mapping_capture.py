#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from datetime import datetime
import os
import time
import threading

class MappingCapture(Node):
    """Captura automática de mapas en formato RoboCup 2026"""
    
    def __init__(self):
        super().__init__('mapping_capture')
        
        # Parámetros
        self.declare_parameter('team_name', 'TeamName')
        self.declare_parameter('mission', 'Prelim1')
        self.declare_parameter('output_dir', '/home/testrobotica/RRL-2026-Robot')
        self.declare_parameter('auto_export_time', 1200)
        
        self.team_name = self.get_parameter('team_name').value
        self.mission = self.get_parameter('mission').value
        self.output_dir = self.get_parameter('output_dir').value
        self.auto_export_time = self.get_parameter('auto_export_time').value
        
        # ============================================
        # CAPTURAR HORA DE INICIO (AL INICIAR EL NODO)
        # ============================================
        self.start_time = datetime.now()
        self.timestamp = self.start_time.strftime('%H-%M-%S')
        
        self.points = []
        self.export_triggered = False
        
        # Suscripciones
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.cloud_callback,
            10
        )
        
        # Timer para auto-export
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info(f'   MAPPING CAPTURE INICIADO')
        self.get_logger().info(f'   Team: {self.team_name}')
        self.get_logger().info(f'   Mission: {self.mission}')
        self.get_logger().info(f'   Start Time: {self.timestamp}')
        self.get_logger().info(f'   Output: {self.output_dir}')
        self.get_logger().info(f'   Auto-export en: {self.auto_export_time}s')
        self.get_logger().info(f'')
        self.get_logger().info(f'Comandos:')
        self.get_logger().info(f'  - Salir: Ctrl+C (guarda automáticamente)')
        self.get_logger().info(f'  - Auto-exporta en {self.auto_export_time}s')

    def cloud_callback(self, msg):
        """Acumula puntos del cloud"""
        try:
            points_list = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
            for point in points_list:
                self.points.append([point[0], point[1], point[2]])
        except Exception as e:
            self.get_logger().error(f'Error en cloud_callback: {e}')

    def timer_callback(self):
        """Verifica si es tiempo de auto-export"""
        elapsed = (datetime.now() - self.start_time).total_seconds()
        
        if elapsed >= self.auto_export_time and not self.export_triggered:
            self.get_logger().warn(f'')
            self.get_logger().warn(f'⏰ TIEMPO LÍMITE ALCANZADO: {self.auto_export_time}s')
            self.get_logger().warn(f'Exportando mapas...')
            self.export_all()
            self.export_triggered = True

    def export_all(self):
        """Exporta todos los archivos necesarios"""
        os.makedirs(self.output_dir, exist_ok=True)
        
        # ============================================
        # EXPORTAR PLY
        # ============================================
        self.save_ply_file()
        
        # ============================================
        # EXPORTAR GeoTIFF (después de que Fast-LIO genere PGM)
        # ============================================
        time.sleep(2)
        self.save_geotiff_file()
        
        # ============================================
        # EXPORTAR CSV PLACEHOLDER
        # ============================================
        self.create_csv_file()

    def save_ply_file(self):
        """Guarda PLY con formato RoboCup 2026"""
        if not self.points:
            self.get_logger().warn('⚠️ No hay puntos para guardar PLY')
            return
        
        points_array = np.array(self.points, dtype=np.float32)
        
        # ============================================
        # NOMBRE CORRECTO
        # ============================================
        filename = f'RoboCup2026-{self.team_name}-{self.mission}-{self.timestamp}-map.ply'
        filepath = os.path.join(self.output_dir, filename)
        
        # Escribir PLY
        with open(filepath, 'w') as f:
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write('comment RoboCup Rescue 2026\n')
            f.write(f'element vertex {len(points_array)}\n')
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')
            f.write('end_header\n')
            for point in points_array:
                f.write(f'{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n')
        
        self.get_logger().info(f'✅ PLY guardado: {filename}')
        self.get_logger().info(f'   Puntos: {len(points_array)}')

    def save_geotiff_file(self):
        """Guarda GeoTIFF"""
        pgm_file = '/home/salvador/robocup2026/maps/mapa_final.pgm'
        
        if not os.path.exists(pgm_file):
            self.get_logger().warn(f'⚠️ PGM no encontrado: {pgm_file}')
            return
        
        try:
            from generate_geotiff import GeoTIFFGenerator
            
            generator = GeoTIFFGenerator(
                pgm_file,
                self.output_dir,
                self.team_name,
                self.mission
            )
            generator.create_geotiff()
            self.get_logger().info(f'✅ GeoTIFF generado correctamente')
        except Exception as e:
            self.get_logger().error(f'Error en GeoTIFF: {e}')

    def create_csv_file(self):
        """Crea CSV con formato RoboCup 2026"""
        # ============================================
        # NOMBRE CORRECTO
        # ============================================
        filename = f'RoboCup2026-{self.team_name}-{self.mission}-{self.timestamp}-pois.csv'
        filepath = os.path.join(self.output_dir, filename)
        
        with open(filepath, 'w') as f:
            # Header del CSV (formato RoboCup)
            f.write('"pois"\n')
            f.write('"1.3"\n')
            f.write(f'"{self.team_name}"\n')
            f.write('"YourCountry"\n')
            f.write(f'"{self.start_time.strftime("%Y-%m-%d")}"\n')
            f.write(f'"{self.timestamp}"\n')
            f.write(f'"{self.mission}"\n')
            
            # Header de datos
            f.write('detection,time,type,name,x,y,z,robot,mode\n')
            
            # Por ahora vacío (se llenará con object_detector más tarde)
        
        self.get_logger().info(f'✅ CSV generado: {filename}')

def main(args=None):
    rclpy.init(args=args)
    node = MappingCapture()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('')
        node.get_logger().info('⏸ Guardando mapas antes de salir...')
        node.export_all()
        time.sleep(2)
        node.get_logger().info('✓ Mapas guardados. Saliendo.')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()