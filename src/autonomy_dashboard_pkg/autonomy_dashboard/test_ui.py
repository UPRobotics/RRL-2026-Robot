#!/usr/bin/env python3
"""
Script de prueba para la interfaz sin necesidad de ROS
Útil para desarrollo y testing local
"""

import sys
import numpy as np
from datetime import datetime, timedelta

# Importa PyQt5
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel)
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont

# Importa widgets del dashboard
from autonomy_dashboard.widgets.map_viewer_2d import Map2DViewer
from autonomy_dashboard.widgets.cloud_viewer_3d import Cloud3DViewer
from autonomy_dashboard.widgets.status_panel import StatusPanel

# Mock para OccupancyGrid
class MockOccupancyGrid:
    def __init__(self, width=100, height=100):
        self.info = MockInfo()
        self.info.width = width
        self.info.height = height
        self.info.resolution = 0.05

        # Crea grid de prueba con patrón de pared
        self.data = []
        for i in range(height):
            for j in range(width):
                if i < 10 or i > 90 or j < 10 or j > 90:
                    self.data.append(100)  # Pared
                elif (i - 50)**2 + (j - 50)**2 < 100:
                    self.data.append(0)  # Libre
                else:
                    self.data.append(-1)  # Desconocido

        self.data = self.data[:width * height]

class MockInfo:
    def __init__(self):
        self.width = 100
        self.height = 100
        self.resolution = 0.05
        self.origin = MockPose()

class MockPose:
    def __init__(self):
        self.position = MockPosition()

class MockPosition:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0

class TestDashboardUI(QMainWindow):
    """UI de prueba sin dependencias de ROS"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle('🤖 Autonomy Dashboard - TEST MODE')
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
            }
            QPushButton:hover {
                background-color: #2ea043;
            }
        """)

        self.init_ui()
        self.start_test_data()

    def init_ui(self):
        """Inicializa la interfaz"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(10, 10, 10, 10)

        # Header
        header = QLabel('🤖 Autonomy Dashboard - TEST MODE (sin ROS)')
        header.setFont(QFont('Arial', 14, QFont.Bold))
        header.setStyleSheet("color: #ffaa00;")
        main_layout.addWidget(header)

        # Content
        content = QHBoxLayout()

        # Left: Maps
        left = QVBoxLayout()

        self.map_2d = Map2DViewer()
        left.addWidget(QLabel('2D Map'), 0)
        left.addWidget(self.map_2d, 1)

        self.cloud_3d = Cloud3DViewer()
        left.addWidget(QLabel('3D Cloud'), 0)
        left.addWidget(self.cloud_3d, 1)

        content.addLayout(left, 2)

        # Right: Status + Buttons
        right = QVBoxLayout()

        self.status = StatusPanel()
        right.addWidget(self.status, 2)

        buttons = QVBoxLayout()

        btn_gen_cloud = QPushButton('Generate Test Cloud')
        btn_gen_cloud.clicked.connect(self.generate_test_cloud)
        buttons.addWidget(btn_gen_cloud)

        btn_update = QPushButton('Update Grid')
        btn_update.clicked.connect(self.update_test_grid)
        buttons.addWidget(btn_update)

        btn_move = QPushButton('Move Robot')
        btn_move.clicked.connect(self.move_robot)
        buttons.addWidget(btn_move)

        right.addLayout(buttons)
        right.addStretch()

        content.addLayout(right, 1)

        main_layout.addLayout(content, 1)
        central_widget.setLayout(main_layout)

    def start_test_data(self):
        """Inicia datos de prueba"""
        self.start_time = datetime.now()
        self.robot_pose = {'x': 0.5, 'y': 0.5, 'theta': 0.0}
        self.grid = MockOccupancyGrid()

        # Actualiza vistas
        self.map_2d.update_grid(self.grid)
        self.map_2d.update_pose(self.robot_pose)

        # Timer para actualizar
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_test_status)
        self.timer.start(500)

    def update_test_status(self):
        """Actualiza estado de prueba"""
        elapsed = (datetime.now() - self.start_time).total_seconds()

        status_data = {
            'pose': self.robot_pose,
            'mapped_area': min(elapsed / 600 * 100, 100),  # Progresa en 10 min
            'cloud_points': int(1000 + elapsed * 100),
            'grid': self.grid,
            'autonomy_mode': 'MAPPING',
            'elapsed_time': int(elapsed),
            'max_time': 1200,
            'objects': [],
            'team': 'TestTeam',
            'mission': 'Test'
        }

        self.status.update_status(status_data)

    def generate_test_cloud(self):
        """Genera nube de puntos de prueba"""
        # Nube en forma de esfera
        n_points = 5000
        theta = np.random.uniform(0, 2*np.pi, n_points)
        phi = np.random.uniform(0, np.pi, n_points)
        r = np.random.uniform(0, 2, n_points)

        x = r * np.sin(phi) * np.cos(theta)
        y = r * np.sin(phi) * np.sin(theta)
        z = r * np.cos(phi) + 1.5

        points = list(zip(x, y, z))
        self.cloud_3d.update_cloud(points)

        self.statusBar().showMessage('Cloud generada: 5000 puntos')

    def update_test_grid(self):
        """Actualiza grid de prueba"""
        self.grid = MockOccupancyGrid()
        self.map_2d.update_grid(self.grid)
        self.statusBar().showMessage('Grid actualizado')

    def move_robot(self):
        """Mueve robot de prueba"""
        self.robot_pose['x'] += 0.5
        self.robot_pose['y'] += 0.3
        self.robot_pose['theta'] += 0.2
        self.map_2d.update_pose(self.robot_pose)
        self.statusBar().showMessage(
            f"Robot en X={self.robot_pose['x']:.2f}, Y={self.robot_pose['y']:.2f}"
        )

def main():
    app = QApplication(sys.argv)
    ui = TestDashboardUI()
    ui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
