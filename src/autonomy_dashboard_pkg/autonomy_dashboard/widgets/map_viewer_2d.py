#!/usr/bin/env python3
"""
Widget para visualizar mapa 2D de SLAM en PyQt5
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPainter, QImage, QPixmap, QColor, QPen, QFont
from PyQt5.QtCore import pyqtSignal
import numpy as np
from nav_msgs.msg import OccupancyGrid


class Map2DViewer(QWidget):
    """Widget para mostrar mapa 2D con posición del robot"""

    pose_updated = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.occupancy_grid = None
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.scale = 50  # píxeles por metro
        self.setMinimumSize(400, 400)
        self.setStyleSheet("background-color: #1a1a1a;")

    def update_grid(self, grid):
        """Actualiza el grid de ocupancia"""
        if grid is not None:
            self.occupancy_grid = grid
            self.update()

    def update_pose(self, pose):
        """Actualiza posición del robot"""
        self.robot_pose = pose
        self.pose_updated.emit(pose)
        self.update()

    def paintEvent(self, event):
        """Dibuja el mapa y el robot"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Fondo
        painter.fillRect(self.rect(), QColor('#1a1a1a'))

        if self.occupancy_grid is None:
            # Dibuja grid de espera
            self.draw_waiting_grid(painter)
            return

        # Dibuja grid y robot
        self.draw_grid(painter)
        self.draw_robot(painter)
        self.draw_info(painter)

    def draw_grid(self, painter):
        """Dibuja el mapa de ocupancia"""
        grid = self.occupancy_grid
        width, height = grid.info.width, grid.info.height
        resolution = grid.info.resolution

        # Crea imagen desde datos
        img_data = np.array(grid.data, dtype=np.uint8).reshape((height, width))

        # Convierte a RGB
        rgb_img = np.zeros((height, width, 3), dtype=np.uint8)

        # Colores: 0=libre (blanco), 100=ocupado (negro), -1=desconocido (gris)
        for i in range(height):
            for j in range(width):
                val = grid.data[i * width + j]
                if val < 0:
                    rgb_img[i, j] = [100, 100, 100]  # Desconocido - gris
                elif val > 50:
                    rgb_img[i, j] = [0, 0, 0]  # Ocupado - negro
                else:
                    rgb_img[i, j] = [255, 255, 255]  # Libre - blanco

        # Convierte a QImage
        h, w, ch = rgb_img.shape
        bytes_per_line = 3 * w
        qt_img = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)

        # Escala y dibuja
        scaled = qt_img.scaledToWidth(int(width * self.scale * resolution))
        x_offset = 50
        y_offset = 50
        painter.drawImage(x_offset, y_offset, scaled)

        # Dibuja grid
        pen = QPen(QColor(50, 50, 50))
        pen.setWidth(1)
        painter.setPen(pen)

        for i in range(0, width, 10):
            x = x_offset + int(i * self.scale * resolution)
            painter.drawLine(x, y_offset, x, y_offset + scaled.height())

        for j in range(0, height, 10):
            y = y_offset + int(j * self.scale * resolution)
            painter.drawLine(x_offset, y, x_offset + scaled.width(), y)

    def draw_robot(self, painter):
        """Dibuja el robot como círculo + flecha"""
        grid = self.occupancy_grid
        resolution = grid.info.resolution

        # Convierte posición a píxeles
        x_pixel = 50 + (self.robot_pose['x'] - grid.info.origin.position.x) / resolution * self.scale
        y_pixel = 50 + (self.robot_pose['y'] - grid.info.origin.position.y) / resolution * self.scale

        # Dibuja círculo del robot
        radius = 15
        painter.setPen(QPen(QColor(0, 255, 0), 2))
        painter.setBrush(QColor(0, 255, 0, 100))
        painter.drawEllipse(int(x_pixel - radius), int(y_pixel - radius), radius * 2, radius * 2)

        # Dibuja dirección (flecha)
        import math
        theta = self.robot_pose['theta']
        arrow_len = 30
        end_x = x_pixel + arrow_len * math.cos(theta)
        end_y = y_pixel - arrow_len * math.sin(theta)

        painter.setPen(QPen(QColor(0, 255, 0), 3))
        painter.drawLine(int(x_pixel), int(y_pixel), int(end_x), int(end_y))

    def draw_info(self, painter):
        """Dibuja información de pose"""
        font = QFont("Arial", 10)
        painter.setFont(font)
        painter.setPen(QColor(255, 255, 255))

        info_text = f"X: {self.robot_pose['x']:.2f}m  Y: {self.robot_pose['y']:.2f}m  Θ: {self.robot_pose['theta']*180/3.14159:.1f}°"
        painter.drawText(10, self.height() - 10, info_text)

    def draw_waiting_grid(self, painter):
        """Dibuja grid de espera cuando no hay datos"""
        pen = QPen(QColor(100, 100, 100))
        pen.setWidth(1)
        painter.setPen(pen)

        spacing = 40
        for i in range(0, self.width(), spacing):
            painter.drawLine(i, 0, i, self.height())
        for j in range(0, self.height(), spacing):
            painter.drawLine(0, j, self.width(), j)

        # Texto
        font = QFont("Arial", 14, QFont.Bold)
        painter.setFont(font)
        painter.setPen(QColor(150, 150, 150))
        painter.drawText(self.rect(), Qt.AlignCenter, "Esperando datos de SLAM...")
