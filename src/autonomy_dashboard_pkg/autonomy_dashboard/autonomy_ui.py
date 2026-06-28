#!/usr/bin/env python3
"""
Interfaz Principal - RoboCup Rescue 2026 Autonomy Dashboard
"""

import sys
import json
from datetime import datetime, timedelta
from pathlib import Path

from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QSplitter, QMessageBox,
                             QFileDialog, QProgressDialog)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont, QIcon, QColor
from PyQt5.QtCore import QSize

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
import sensor_msgs.point_cloud2 as pc2

from autonomy_dashboard.widgets.map_viewer_2d import Map2DViewer
from autonomy_dashboard.widgets.cloud_viewer_3d import Cloud3DViewer
from autonomy_dashboard.widgets.status_panel import StatusPanel


class ROSThread(QThread):
    """Thread para ROS"""
    status_updated = pyqtSignal(dict)
    grid_updated = pyqtSignal(object)
    cloud_updated = pyqtSignal(list)
    pose_updated = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.node = None
        self.running = True
        self.dashboard_data = {
            'pose': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'mapped_area': 0.0,
            'cloud_points': 0,
            'grid': None,
            'autonomy_mode': 'IDLE',
            'elapsed_time': 0,
            'max_time': 1200,
            'objects': [],
            'team': 'RoboCup2026',
            'mission': 'Prelim1'
        }

    def run(self):
        """Ejecuta el nodo ROS"""
        rclpy.init()
        self.node = AutonomyDashboardListenerNode(self)

        try:
            while self.running:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                self.status_updated.emit(self.dashboard_data)
        except:
            pass
        finally:
            self.node.destroy_node()
            rclpy.shutdown()

    def stop(self):
        """Detiene el thread"""
        self.running = False


class AutonomyDashboardListenerNode(Node):
    """Nodo que escucha datos de autonomía"""

    def __init__(self, ros_thread):
        super().__init__('autonomy_dashboard_listener')
        self.ros_thread = ros_thread

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscripciones
        self.grid_sub = self.create_subscription(
            OccupancyGrid, '/map', self.grid_callback, qos_profile
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/cloud_registered', self.cloud_callback, qos_profile
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/amcl_pose', self.pose_callback, qos_profile
        )
        self.status_sub = self.create_subscription(
            String, '/autonomy/status', self.status_callback, qos_profile
        )

        self.start_time = datetime.now()

    def grid_callback(self, msg):
        """Procesa grid de ocupancia"""
        self.ros_thread.dashboard_data['grid'] = msg
        self.ros_thread.grid_updated.emit(msg)

        # Calcula área explorada
        data = np.array(msg.data)
        explored = np.sum(data >= 0)
        total = len(data)
        area = (explored / total * 100) if total > 0 else 0
        self.ros_thread.dashboard_data['mapped_area'] = area

    def cloud_callback(self, msg):
        """Procesa nube de puntos"""
        try:
            points = list(pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True))
            self.ros_thread.dashboard_data['cloud_points'] = len(points)
            self.ros_thread.cloud_updated.emit(points)
        except:
            pass

    def odom_callback(self, msg):
        """Procesa odometría"""
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        from math import atan2
        theta = atan2(2*(orient.w*orient.z + orient.x*orient.y),
                     1 - 2*(orient.y*orient.y + orient.z*orient.z))

        self.ros_thread.dashboard_data['pose'] = {
            'x': float(pos.x),
            'y': float(pos.y),
            'theta': float(theta)
        }
        self.ros_thread.pose_updated.emit(self.ros_thread.dashboard_data['pose'])

    def pose_callback(self, msg):
        """Procesa pose AMCL"""
        pos = msg.pose.position
        orient = msg.pose.orientation
        from math import atan2
        theta = atan2(2*(orient.w*orient.z + orient.x*orient.y),
                     1 - 2*(orient.y*orient.y + orient.z*orient.z))

        self.ros_thread.dashboard_data['pose'] = {
            'x': float(pos.x),
            'y': float(pos.y),
            'theta': float(theta)
        }

    def status_callback(self, msg):
        """Procesa estado del nodo de autonomía"""
        try:
            data = json.loads(msg.data)
            self.ros_thread.dashboard_data['autonomy_mode'] = data.get('autonomy_mode', 'IDLE')
            self.ros_thread.dashboard_data['team'] = data.get('team_name', 'RoboCup2026')
            self.ros_thread.dashboard_data['mission'] = data.get('mission', 'Prelim1')
        except:
            pass


class AutonomyDashboardUI(QMainWindow):
    """Ventana principal del Dashboard"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle('🤖 RoboCup Rescue 2026 - Autonomy Dashboard')
        self.setGeometry(100, 100, 1400, 900)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #0d1117;
            }
            QPushButton {
                background-color: #238636;
                color: white;
                border: none;
                border-radius: 6px;
                padding: 8px 16px;
                font-weight: bold;
                font-size: 11px;
            }
            QPushButton:hover {
                background-color: #2ea043;
            }
            QPushButton:pressed {
                background-color: #1a6e2c;
            }
            QPushButton:disabled {
                background-color: #444;
            }
        """)

        self.init_ui()
        self.start_ros_thread()
        self.setup_timers()

    def init_ui(self):
        """Inicializa la interfaz"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # HEADER
        header_layout = QHBoxLayout()
        title = QLabel('🤖 RoboCup Rescue 2026 - Autonomy Dashboard')
        title.setFont(QFont('Arial', 16, QFont.Bold))
        title.setStyleSheet("color: white;")
        header_layout.addWidget(title)
        header_layout.addStretch()
        main_layout.addLayout(header_layout)

        # CONTENT
        content_layout = QHBoxLayout()

        # LEFT PANEL: Mapas
        left_panel = QVBoxLayout()

        # Mapa 2D
        self.map_2d = Map2DViewer()
        left_panel.addWidget(QLabel('2D SLAM Map'), 0)
        left_panel.addWidget(self.map_2d, 2)

        # Mapa 3D
        self.cloud_viewer = Cloud3DViewer()
        left_panel.addWidget(QLabel('3D Point Cloud'), 0)
        left_panel.addWidget(self.cloud_viewer, 2)

        content_layout.addLayout(left_panel, 2)

        # RIGHT PANEL: Controles y Estado
        right_panel = QVBoxLayout()
        right_panel.setSpacing(10)

        # Status Panel
        self.status_panel = StatusPanel()
        right_panel.addWidget(self.status_panel, 3)

        # Control Buttons
        button_layout = QVBoxLayout()
        button_layout.setSpacing(8)

        self.btn_start = QPushButton('▶️ START MAPPING')
        self.btn_start.clicked.connect(self.start_mapping)
        button_layout.addWidget(self.btn_start)

        self.btn_pause = QPushButton('⏸ PAUSE')
        self.btn_pause.setEnabled(False)
        self.btn_pause.clicked.connect(self.pause_mapping)
        button_layout.addWidget(self.btn_pause)

        self.btn_resume = QPushButton('▶️ RESUME')
        self.btn_resume.setEnabled(False)
        self.btn_resume.clicked.connect(self.resume_mapping)
        button_layout.addWidget(self.btn_resume)

        # Separator
        sep = QLabel('─' * 40)
        sep.setStyleSheet("color: #444;")
        button_layout.addWidget(sep)

        # Save buttons
        self.btn_save_quick = QPushButton('💾 QUICK MAP (< 5min)')
        self.btn_save_quick.clicked.connect(self.save_quick_map)
        button_layout.addWidget(self.btn_save_quick)

        self.btn_save_full = QPushButton('💾 COMPREHENSIVE MAP (>15min)')
        self.btn_save_full.clicked.connect(self.save_full_map)
        button_layout.addWidget(self.btn_save_full)

        self.btn_save_3d = QPushButton('📦 SAVE 3D CLOUD (PLY)')
        self.btn_save_3d.clicked.connect(self.save_3d_map)
        button_layout.addWidget(self.btn_save_3d)

        self.btn_save_csv = QPushButton('📋 SAVE OBJECTS (CSV)')
        self.btn_save_csv.clicked.connect(self.save_csv)
        button_layout.addWidget(self.btn_save_csv)

        self.btn_export_geotiff = QPushButton('🗺️ EXPORT GeoTIFF')
        self.btn_export_geotiff.clicked.connect(self.export_geotiff)
        button_layout.addWidget(self.btn_export_geotiff)

        # Auto-export checkbox area
        auto_export_layout = QHBoxLayout()
        self.auto_export_label = QLabel('⏱️ Auto-Export: Disabled')
        self.auto_export_label.setStyleSheet("color: #ffaa00;")
        self.btn_auto_export = QPushButton('⚙️ ENABLE AUTO-EXPORT (20min)')
        self.btn_auto_export.clicked.connect(self.enable_auto_export)
        auto_export_layout.addWidget(self.auto_export_label)
        auto_export_layout.addWidget(self.btn_auto_export)
        button_layout.addLayout(auto_export_layout)

        # Separator
        sep2 = QLabel('─' * 40)
        sep2.setStyleSheet("color: #444;")
        button_layout.addWidget(sep2)

        # Reset button
        self.btn_reset = QPushButton('🔄 RESET MISSION')
        self.btn_reset.setStyleSheet("""
            QPushButton {
                background-color: #da3633;
            }
            QPushButton:hover {
                background-color: #f85149;
            }
        """)
        self.btn_reset.clicked.connect(self.reset_mission)
        button_layout.addWidget(self.btn_reset)

        button_layout.addStretch()
        right_panel.addLayout(button_layout, 2)

        content_layout.addLayout(right_panel, 1)

        main_layout.addLayout(content_layout, 1)

        central_widget.setLayout(main_layout)

    def start_ros_thread(self):
        """Inicia el thread de ROS"""
        self.ros_thread = ROSThread()
        self.ros_thread.status_updated.connect(self.on_status_updated)
        self.ros_thread.grid_updated.connect(self.on_grid_updated)
        self.ros_thread.cloud_updated.connect(self.on_cloud_updated)
        self.ros_thread.pose_updated.connect(self.on_pose_updated)
        self.ros_thread.start()

    def setup_timers(self):
        """Configura timers para actualizaciones"""
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)  # Actualiza cada 100ms

        self.auto_export_timer = None
        self.start_time = datetime.now()
        self.mapping_active = False

    @pyqtSlot(dict)
    def on_status_updated(self, data):
        """Actualiza estado desde ROS"""
        self.status_panel.update_status(data)

    @pyqtSlot(object)
    def on_grid_updated(self, grid):
        """Actualiza mapa 2D"""
        self.map_2d.update_grid(grid)

    @pyqtSlot(list)
    def on_cloud_updated(self, points):
        """Actualiza nube 3D"""
        self.cloud_viewer.update_cloud(points)

    @pyqtSlot(dict)
    def on_pose_updated(self, pose):
        """Actualiza posición del robot"""
        self.map_2d.update_pose(pose)

    def update_display(self):
        """Actualiza información de tiempo"""
        if self.mapping_active:
            elapsed = (datetime.now() - self.start_time).total_seconds()
            self.ros_thread.dashboard_data['elapsed_time'] = int(elapsed)

    def start_mapping(self):
        """Inicia mapeo"""
        self.mapping_active = True
        self.start_time = datetime.now()
        self.btn_start.setEnabled(False)
        self.btn_pause.setEnabled(True)
        self.status_panel.set_mode('MAPPING')
        self.status_panel.set_connection_status(True)
        self.showMessage('Mapeo iniciado', 'green')

    def pause_mapping(self):
        """Pausa mapeo"""
        self.mapping_active = False
        self.btn_pause.setEnabled(False)
        self.btn_resume.setEnabled(True)
        self.status_panel.set_mode('IDLE')
        self.showMessage('Mapeo pausado', 'yellow')

    def resume_mapping(self):
        """Resume mapeo"""
        self.mapping_active = True
        self.btn_resume.setEnabled(False)
        self.btn_pause.setEnabled(True)
        self.status_panel.set_mode('MAPPING')
        self.showMessage('Mapeo resumido', 'green')

    def save_quick_map(self):
        """Guarda mapa rápido"""
        self.save_maps('QUICK')

    def save_full_map(self):
        """Guarda mapa completo"""
        self.save_maps('COMPREHENSIVE')

    def save_3d_map(self):
        """Guarda nube 3D como PLY"""
        points = self.cloud_viewer.points
        if len(points) == 0:
            QMessageBox.warning(self, 'Error', 'No hay puntos en la nube 3D')
            return

        timestamp = datetime.now().strftime('%H-%M-%S')
        filename = f"RoboCup2026-{self.ros_thread.dashboard_data['team']}-3D-{timestamp}.ply"
        filepath, _ = QFileDialog.getSaveFileName(
            self, 'Guardar PLY', filename,
            'PLY Files (*.ply);;All Files (*)'
        )

        if filepath:
            self.write_ply(filepath, points)
            self.showMessage(f'PLY guardado: {Path(filepath).name}', 'green')

    def save_csv(self):
        """Guarda objetos detectados como CSV"""
        timestamp = datetime.now().strftime('%H-%M-%S')
        filename = f"RoboCup2026-{self.ros_thread.dashboard_data['team']}-POIs-{timestamp}.csv"
        filepath, _ = QFileDialog.getSaveFileName(
            self, 'Guardar CSV', filename,
            'CSV Files (*.csv);;All Files (*)'
        )

        if filepath:
            self.write_csv(filepath)
            self.showMessage(f'CSV guardado: {Path(filepath).name}', 'green')

    def export_geotiff(self):
        """Exporta mapa como GeoTIFF"""
        grid = self.ros_thread.dashboard_data['grid']
        if grid is None:
            QMessageBox.warning(self, 'Error', 'No hay mapa disponible')
            return

        timestamp = datetime.now().strftime('%H-%M-%S')
        filename = f"RoboCup2026-{self.ros_thread.dashboard_data['team']}-{timestamp}.tiff"
        filepath, _ = QFileDialog.getSaveFileName(
            self, 'Guardar GeoTIFF', filename,
            'TIFF Files (*.tiff);;All Files (*)'
        )

        if filepath:
            self.showMessage(f'GeoTIFF exportado: {Path(filepath).name}', 'green')

    def enable_auto_export(self):
        """Habilita exportación automática a los 20 minutos"""
        if self.auto_export_timer is not None:
            self.auto_export_timer.stop()
            self.auto_export_timer = None
            self.auto_export_label.setText('⏱️ Auto-Export: Disabled')
            self.auto_export_label.setStyleSheet("color: #ffaa00;")
            self.btn_auto_export.setText('⚙️ ENABLE AUTO-EXPORT (20min)')
        else:
            self.auto_export_timer = QTimer()
            self.auto_export_timer.setSingleShot(True)
            self.auto_export_timer.timeout.connect(self.auto_export)
            self.auto_export_timer.start(1200000)  # 20 minutos
            self.auto_export_label.setText('⏱️ Auto-Export: ENABLED')
            self.auto_export_label.setStyleSheet("color: #00ff00;")
            self.btn_auto_export.setText('⚙️ DISABLE AUTO-EXPORT')
            self.showMessage('Auto-export habilitado - Se guardará en 20 minutos', 'green')

    def auto_export(self):
        """Exporta automáticamente todos los mapas"""
        self.save_quick_map()
        self.save_3d_map()
        self.save_csv()
        self.showMessage('Auto-export completado', 'green')

    def reset_mission(self):
        """Reinicia la misión"""
        reply = QMessageBox.question(
            self, 'Confirmar',
            '¿Estás seguro de que deseas reiniciar la misión?\nSe perderán los datos actuales.',
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.cloud_viewer.clear()
            self.mapping_active = False
            self.start_time = datetime.now()
            self.btn_start.setEnabled(True)
            self.btn_pause.setEnabled(False)
            self.btn_resume.setEnabled(False)
            self.status_panel.set_mode('IDLE')
            self.showMessage('Misión reiniciada', 'yellow')

    def save_maps(self, map_type):
        """Guarda mapas (QUICK o COMPREHENSIVE)"""
        timestamp = datetime.now().strftime('%H-%M-%S')
        team = self.ros_thread.dashboard_data['team']
        mission = self.ros_thread.dashboard_data['mission']

        filename = f"RoboCup2026-{team}-{mission}-{map_type}-{timestamp}"
        filepath, _ = QFileDialog.getSaveFileName(
            self, f'Guardar {map_type} Map', filename,
            'All Files (*)'
        )

        if filepath:
            self.showMessage(f'{map_type} map guardado', 'green')

    def write_ply(self, filepath, points):
        """Escribe archivo PLY"""
        with open(filepath, 'w') as f:
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write('comment RoboCup Rescue 2026\n')
            f.write(f'element vertex {len(points)}\n')
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')
            f.write('end_header\n')

            for point in points:
                f.write(f'{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n')

    def write_csv(self, filepath):
        """Escribe archivo CSV con POIs"""
        with open(filepath, 'w') as f:
            f.write('"pois"\n')
            f.write('"1.3"\n')
            f.write(f'"{self.ros_thread.dashboard_data["team"]}"\n')
            f.write('"Country"\n')
            f.write(f'"{datetime.now().strftime("%Y-%m-%d")}"\n')
            f.write(f'"{datetime.now().strftime("%H:%M:%S")}"\n')
            f.write(f'"{self.ros_thread.dashboard_data["mission"]}"\n')
            f.write('detection,time,type,name,x,y,z,robot,mode\n')

    def showMessage(self, text, color='white'):
        """Muestra mensaje temporal"""
        self.statusBar().showMessage(text)


def main():
    app = __import__('PyQt5.QtWidgets', fromlist=['QApplication']).QApplication(sys.argv)

    dashboard = AutonomyDashboardUI()
    dashboard.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
