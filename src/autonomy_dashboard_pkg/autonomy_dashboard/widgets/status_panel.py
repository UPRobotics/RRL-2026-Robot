#!/usr/bin/env python3
"""
Panel de estado para Autonomy Dashboard
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QProgressBar
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QFont
from datetime import timedelta


class StatusPanel(QWidget):
    """Widget para mostrar estado de autonomía"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()

    def init_ui(self):
        """Inicializa la interfaz"""
        layout = QVBoxLayout()
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        self.setLayout(layout)

        # Estilo
        self.setStyleSheet("""
            QLabel {
                color: white;
                font-size: 11px;
            }
            QProgressBar {
                border: 2px solid #555;
                border-radius: 5px;
                background-color: #222;
            }
            QProgressBar::chunk {
                background-color: #00ff00;
            }
        """)

        # Row 1: Modo y Estado
        row1 = QHBoxLayout()
        self.mode_label = QLabel('🤖 MODO: IDLE')
        self.mode_label.setFont(QFont('Arial', 12, QFont.Bold))
        self.status_label = QLabel('✓ Conectado')
        self.status_label.setStyleSheet("color: #00ff00;")
        row1.addWidget(self.mode_label)
        row1.addStretch()
        row1.addWidget(self.status_label)
        layout.addLayout(row1)

        # Row 2: Posición
        row2 = QHBoxLayout()
        self.pose_label = QLabel('📍 Pose: X=0.00m  Y=0.00m  Θ=0.0°')
        self.pose_label.setFont(QFont('Courier', 10))
        row2.addWidget(self.pose_label)
        layout.addLayout(row2)

        # Row 3: Área explorada
        row3 = QVBoxLayout()
        self.area_label = QLabel('🗺️  Área Explorada: 0%')
        self.area_progress = QProgressBar()
        self.area_progress.setValue(0)
        row3.addWidget(self.area_label)
        row3.addWidget(self.area_progress)
        layout.addLayout(row3)

        # Row 4: Puntos nube
        row4 = QHBoxLayout()
        self.points_label = QLabel('☁️  Puntos Cloud: 0')
        self.points_label.setFont(QFont('Arial', 11))
        row4.addWidget(self.points_label)
        row4.addStretch()
        self.grid_label = QLabel('Grid: None')
        row4.addWidget(self.grid_label)
        layout.addLayout(row4)

        # Row 5: Tiempo
        row5 = QHBoxLayout()
        self.time_label = QLabel('⏱️  Tiempo: 00:00 / 20:00')
        self.time_label.setFont(QFont('Courier', 11, QFont.Bold))
        row5.addWidget(self.time_label)
        row5.addStretch()
        self.objects_label = QLabel('Objetos: 0')
        row5.addWidget(self.objects_label)
        layout.addLayout(row5)

        # Row 6: Información extra
        row6 = QHBoxLayout()
        self.team_label = QLabel('Team: RoboCup2026')
        self.mission_label = QLabel('Mission: Prelim1')
        row6.addWidget(self.team_label)
        row6.addStretch()
        row6.addWidget(self.mission_label)
        layout.addLayout(row6)

        layout.addStretch()

    def update_status(self, data):
        """Actualiza el estado desde datos del nodo"""
        if not data:
            return

        # Modo
        mode = data.get('autonomy_mode', 'IDLE')
        mode_icons = {
            'IDLE': '⚪',
            'MAPPING': '🟢',
            'EXPLORING': '🟡',
            'NAVIGATING': '🔵',
            'FAULT': '🔴'
        }
        icon = mode_icons.get(mode, '⚪')
        self.mode_label.setText(f'{icon} MODO: {mode}')

        # Posición
        pose = data.get('pose', {})
        self.pose_label.setText(
            f"📍 Pose: X={pose.get('x', 0):.2f}m  Y={pose.get('y', 0):.2f}m  Θ={pose.get('theta', 0)*180/3.14159:.1f}°"
        )

        # Área explorada
        area = data.get('mapped_area', 0)
        self.area_label.setText(f'🗺️  Área Explorada: {area:.1f}%')
        self.area_progress.setValue(int(area))

        # Puntos
        cloud = data.get('cloud_points', 0)
        self.points_label.setText(f'☁️  Puntos Cloud: {cloud:,}')

        # Tiempo
        elapsed = data.get('elapsed_time', 0)
        total = data.get('max_time', 1200)
        elapsed_str = str(timedelta(seconds=elapsed)).split('.')[0]
        total_str = str(timedelta(seconds=total)).split('.')[0]
        self.time_label.setText(f'⏱️  Tiempo: {elapsed_str} / {total_str}')

        # Objetos
        objects = data.get('objects', [])
        self.objects_label.setText(f'Objetos: {len(objects)}')

        # Team y Mission
        self.team_label.setText(f"Team: {data.get('team', 'RoboCup2026')}")
        self.mission_label.setText(f"Mission: {data.get('mission', 'Prelim1')}")

    def set_connection_status(self, connected):
        """Actualiza estado de conexión"""
        if connected:
            self.status_label.setText('✓ Conectado')
            self.status_label.setStyleSheet("color: #00ff00;")
        else:
            self.status_label.setText('✗ Desconectado')
            self.status_label.setStyleSheet("color: #ff0000;")

    def set_mode(self, mode):
        """Cambia el modo mostrado"""
        mode_icons = {
            'IDLE': '⚪',
            'MAPPING': '🟢',
            'EXPLORING': '🟡',
            'NAVIGATING': '🔵',
            'FAULT': '🔴'
        }
        icon = mode_icons.get(mode, '⚪')
        self.mode_label.setText(f'{icon} MODO: {mode}')
