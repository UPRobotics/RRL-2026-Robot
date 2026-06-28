#!/usr/bin/env python3
"""
RoboCup Rescue 2026 - Autonomy Dashboard Node
Agrega datos de Fast-LIO, Nav2, y sensores para visualización en tiempo real
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32, Int32
import numpy as np
from datetime import datetime
import json
import os


class AutonomyDashboardNode(Node):
    """Nodo que agrega datos de autonomía para el dashboard"""

    def __init__(self):
        super().__init__('autonomy_dashboard_node')

        # Parámetros
        self.declare_parameter('team_name', 'RoboCup2026')
        self.declare_parameter('mission', 'Prelim1')
        self.declare_parameter('output_dir', '/home/testrobotica/RRL-2026-Robot/maps')
        self.declare_parameter('max_mapping_time', 1200)  # 20 minutos

        self.team_name = self.get_parameter('team_name').value
        self.mission = self.get_parameter('mission').value
        self.output_dir = self.get_parameter('output_dir').value
        self.max_mapping_time = self.get_parameter('max_mapping_time').value

        # Estado interno
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.cloud_points = []
        self.occupancy_grid = None
        self.start_time = datetime.now()
        self.mapped_area = 0.0
        self.autonomy_mode = 'IDLE'
        self.detected_objects = []

        # QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscripciones
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.cloud_callback,
            qos_profile
        )

        self.grid_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.grid_callback,
            qos_profile
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/amcl_pose',
            self.pose_callback,
            qos_profile
        )

        # Publicadores de estado
        self.status_pub = self.create_publisher(String, '/autonomy/status', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/autonomy/robot_pose', 10)
        self.progress_pub = self.create_publisher(Float32, '/autonomy/mapped_area_percent', 10)
        self.object_pub = self.create_publisher(String, '/autonomy/detected_objects', 10)

        # Timer para publicar estado
        self.timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('✓ Autonomy Dashboard Node iniciado')
        self.get_logger().info(f'  Team: {self.team_name}')
        self.get_logger().info(f'  Mission: {self.mission}')
        self.get_logger().info(f'  Output Dir: {self.output_dir}')
        self.get_logger().info(f'  Max Mapping Time: {self.max_mapping_time}s')

    def cloud_callback(self, msg):
        """Procesa nube de puntos de Fast-LIO"""
        try:
            # Extrae puntos XYZ
            import sensor_msgs.point_cloud2 as pc2
            points = list(pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True))

            if len(points) > 0:
                self.cloud_points = points
                self.get_logger().debug(f'Cloud: {len(points)} puntos')
        except Exception as e:
            self.get_logger().error(f'Error en cloud_callback: {e}')

    def grid_callback(self, msg):
        """Procesa mapa de ocupancia (SLAM)"""
        try:
            self.occupancy_grid = msg

            # Calcula área explorada
            data = np.array(msg.data)
            explored = np.sum(data >= 0)  # -1 = desconocido
            total = len(data)
            self.mapped_area = (explored / total * 100) if total > 0 else 0.0

        except Exception as e:
            self.get_logger().error(f'Error en grid_callback: {e}')

    def odom_callback(self, msg):
        """Procesa odometría"""
        try:
            pos = msg.pose.pose.position
            orient = msg.pose.pose.orientation

            # Convierte quaternión a euler
            from math import atan2, asin
            x = orient.x
            y = orient.y
            z = orient.z
            w = orient.w

            theta = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

            self.robot_pose = {
                'x': float(pos.x),
                'y': float(pos.y),
                'theta': float(theta)
            }
        except Exception as e:
            self.get_logger().error(f'Error en odom_callback: {e}')

    def pose_callback(self, msg):
        """Procesa pose AMCL"""
        try:
            pos = msg.pose.position
            orient = msg.pose.orientation

            from math import atan2
            theta = atan2(2*(orient.w*orient.z + orient.x*orient.y),
                         1 - 2*(orient.y*orient.y + orient.z*orient.z))

            self.robot_pose = {
                'x': float(pos.x),
                'y': float(pos.y),
                'theta': float(theta)
            }
        except Exception as e:
            self.get_logger().error(f'Error en pose_callback: {e}')

    def publish_status(self):
        """Publica estado actual del dashboard"""
        try:
            # Calcula tiempo transcurrido
            elapsed = (datetime.now() - self.start_time).total_seconds()

            # Status JSON
            status_data = {
                'timestamp': datetime.now().isoformat(),
                'team_name': self.team_name,
                'mission': self.mission,
                'autonomy_mode': self.autonomy_mode,
                'robot_pose': self.robot_pose,
                'mapped_area': round(self.mapped_area, 2),
                'cloud_points': len(self.cloud_points),
                'elapsed_time': int(elapsed),
                'max_time': self.max_mapping_time,
                'detected_objects': len(self.detected_objects)
            }

            status_msg = String()
            status_msg.data = json.dumps(status_data)
            self.status_pub.publish(status_msg)

            # Publica progreso
            progress_msg = Float32()
            progress_msg.data = self.mapped_area
            self.progress_pub.publish(progress_msg)

        except Exception as e:
            self.get_logger().error(f'Error en publish_status: {e}')

    def set_autonomy_mode(self, mode):
        """Cambia modo de autonomía"""
        valid_modes = ['IDLE', 'MAPPING', 'EXPLORING', 'NAVIGATING', 'FAULT']
        if mode in valid_modes:
            self.autonomy_mode = mode
            self.get_logger().info(f'Autonomy Mode: {mode}')

    def get_dashboard_data(self):
        """Retorna datos formateados para el dashboard"""
        elapsed = (datetime.now() - self.start_time).total_seconds()

        return {
            'pose': self.robot_pose,
            'mapped_area': self.mapped_area,
            'cloud_points': len(self.cloud_points),
            'grid': self.occupancy_grid,
            'autonomy_mode': self.autonomy_mode,
            'elapsed_time': int(elapsed),
            'max_time': self.max_mapping_time,
            'objects': self.detected_objects,
            'team': self.team_name,
            'mission': self.mission
        }


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyDashboardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Apagando Autonomy Dashboard Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
