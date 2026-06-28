#!/usr/bin/env python3
"""
Widget para visualizar nube de puntos 3D en PyQt5
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtCore import Qt, pyqtSignal
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D


class Cloud3DViewer(QWidget):
    """Widget para mostrar nube de puntos 3D"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.points = np.array([]).reshape(0, 3)
        self.init_ui()

    def init_ui(self):
        """Inicializa la interfaz"""
        layout = QVBoxLayout()
        self.setLayout(layout)

        # Crea figura matplotlib
        self.figure = Figure(figsize=(6, 6), dpi=100, facecolor='#1a1a1a')
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111, projection='3d')

        layout.addWidget(self.canvas)
        self.setMinimumSize(400, 400)

        # Dibuja nube inicial vacía
        self.draw_cloud()

    def update_cloud(self, points):
        """Actualiza la nube de puntos"""
        if isinstance(points, list) and len(points) > 0:
            self.points = np.array(points, dtype=np.float32)
        self.draw_cloud()

    def draw_cloud(self):
        """Dibuja la nube de puntos"""
        self.ax.clear()
        self.ax.set_facecolor('#1a1a1a')

        if len(self.points) > 0:
            # Downsampling si hay demasiados puntos
            if len(self.points) > 50000:
                indices = np.random.choice(len(self.points), 50000, replace=False)
                pts = self.points[indices]
            else:
                pts = self.points

            # Colorea por altura (Z)
            if len(pts) > 0:
                z_min = pts[:, 2].min()
                z_max = pts[:, 2].max()
                z_normalized = (pts[:, 2] - z_min) / (z_max - z_min + 1e-6)

                scatter = self.ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                                        c=z_normalized, cmap='viridis',
                                        s=1, alpha=0.6)

        # Configuración de ejes
        self.ax.set_xlabel('X (m)', color='white')
        self.ax.set_ylabel('Y (m)', color='white')
        self.ax.set_zlabel('Z (m)', color='white')
        self.ax.tick_params(colors='white')
        self.ax.grid(True, alpha=0.3)

        # Límites
        if len(self.points) > 0:
            margin = 1.0
            self.ax.set_xlim(self.points[:, 0].min() - margin, self.points[:, 0].max() + margin)
            self.ax.set_ylim(self.points[:, 1].min() - margin, self.points[:, 1].max() + margin)
            self.ax.set_zlim(0, self.points[:, 2].max() + 0.5)

        self.canvas.draw()

    def get_points_count(self):
        """Retorna número de puntos"""
        return len(self.points)

    def get_bounds(self):
        """Retorna límites de la nube"""
        if len(self.points) == 0:
            return None

        return {
            'x_min': float(self.points[:, 0].min()),
            'x_max': float(self.points[:, 0].max()),
            'y_min': float(self.points[:, 1].min()),
            'y_max': float(self.points[:, 1].max()),
            'z_min': float(self.points[:, 2].min()),
            'z_max': float(self.points[:, 2].max()),
        }

    def clear(self):
        """Limpia la nube"""
        self.points = np.array([]).reshape(0, 3)
        self.draw_cloud()
