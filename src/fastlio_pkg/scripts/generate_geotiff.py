#!/usr/bin/env python3

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import os
from datetime import datetime
from pathlib import Path

class GeoTIFFGenerator:
    """Genera GeoTIFF con formato específico RoboCup Rescue 2026"""
    
    def __init__(self, pgm_file, output_dir, team_name='RoboCup2026', mission_name='Prelim1'):
        self.pgm_file = pgm_file
        self.output_dir = output_dir
        self.team_name = team_name
        self.mission_name = mission_name
        self.resolution = 0.05  # metros por pixel
        
    def load_pgm_map(self):
        """Carga mapa PGM"""
        # Leer con OpenCV
        img = cv2.imread(self.pgm_file, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"No se pudo cargar {self.pgm_file}")
        return img
    
    def create_geotiff(self, pgm_map=None, robot_trajectory=None, objects_detected=None):
        """
        Crea GeoTIFF con formato RoboCup
        
        Args:
            pgm_map: numpy array del mapa PGM
            robot_trajectory: lista de (x, y) del camino del robot
            objects_detected: dict con ubicaciones de objetos
        """
        
        if pgm_map is None:
            pgm_map = self.load_pgm_map()
        
        # Convertir a RGB para agregar colores
        height, width = pgm_map.shape
        rgb_map = cv2.cvtColor(pgm_map, cv2.COLOR_GRAY2RGB)
        
        # Crear imagen con espacios para elementos visuales
        canvas_width = width + 400  # Espacio para escala y orientación
        canvas_height = height + 300  # Espacio para header
        
        canvas = Image.new('RGB', (canvas_width, canvas_height), color='white')
        draw = ImageDraw.Draw(canvas)
        
        # 1. PEGAR MAPA en posición (200, 150)
        map_pil = Image.fromarray(rgb_map)
        canvas.paste(map_pil, (200, 150))
        
        # 2. AGREGAR GRID DE EXPLORACIÓN
        self.add_exploration_grid(canvas, 200, 150, width, height)
        
        # 3. AGREGAR ESCALA (1 metro = línea de 1cm visual)
        self.add_scale(draw, canvas_width - 150, canvas_height - 100)
        
        # 4. AGREGAR ORIENTACIÓN (X Y)
        self.add_orientation(draw, canvas_width - 150, canvas_height - 50)
        
        # 5. NOMBRE DEL ARCHIVO (esquina superior izquierda)
        self.add_header(draw, canvas_width, canvas_height)
        
        # 6. AGREGAR CAMINO DEL ROBOT (si disponible)
        if robot_trajectory:
            self.add_robot_path(canvas, robot_trajectory, 200, 150)
        
        # 7. AGREGAR OBJETOS DETECTADOS (si disponible)
        if objects_detected:
            self.add_detected_objects(canvas, objects_detected, 200, 150)
        
        # Guardar como TIFF (más cercano a GeoTIFF sin dependencias geográficas)
        timestamp = datetime.now().strftime('%H-%M-%S')
        filename = f'{self.team_name}-{self.mission_name}-{timestamp}.tiff'
        filepath = os.path.join(self.output_dir, filename)
        
        os.makedirs(self.output_dir, exist_ok=True)
        canvas.save(filepath, format='TIFF', compression='none')
        
        print(f"✓ GeoTIFF generado: {filepath}")
        print(f"  Tamaño: {canvas_width}x{canvas_height} pixels")
        print(f"  Resolución: 0.05 m/pixel")
        
        return filepath

    def add_exploration_grid(self, canvas, x_offset, y_offset, width, height):
        """Agrega grid de áreas exploradas (50cm) y no exploradas (100cm)"""
        draw = ImageDraw.Draw(canvas)
        
        # Grid de no explorado: 100cm = 2000 pixels a 0.05m/px (200 pixels)
        unexplored_grid_px = int(1.0 / self.resolution)  # 20 pixels
        color_light = (226, 226, 227)
        color_dark = (237, 237, 238)
        
        # Dibujar checkerboard detrás del mapa (en área no explorada)
        for i in range(0, width * 2, unexplored_grid_px):
            for j in range(0, height * 2, unexplored_grid_px):
                color = color_light if ((i + j) // unexplored_grid_px) % 2 == 0 else color_dark
                draw.rectangle(
                    [(x_offset + i, y_offset + j), 
                     (x_offset + i + unexplored_grid_px, y_offset + j + unexplored_grid_px)],
                    fill=color
                )
        
        # Grid de explorado: 50cm = 100 pixels (negro, 1px de grosor)
        explored_grid_px = int(0.5 / self.resolution)  # 10 pixels
        for i in range(0, width, explored_grid_px):
            draw.line([(x_offset + i, y_offset), (x_offset + i, y_offset + height)], 
                     fill=(190, 190, 191), width=1)
        for j in range(0, height, explored_grid_px):
            draw.line([(x_offset, y_offset + j), (x_offset + width, y_offset + j)], 
                     fill=(190, 190, 191), width=1)

    def add_scale(self, draw, x, y):
        """Agrega escala visual (1 metro = línea)"""
        # 1 metro a 0.05m/px = 20 pixels
        scale_px = int(1.0 / self.resolution)  # 20 pixels
        
        # Línea horizontal
        draw.line([(x, y), (x + scale_px, y)], fill=(0, 50, 140), width=3)
        
        # Marcas verticales
        draw.line([(x, y - 5), (x, y + 5)], fill=(0, 50, 140), width=2)
        draw.line([(x + scale_px, y - 5), (x + scale_px, y + 5)], fill=(0, 50, 140), width=2)
        
        # Texto "1m"
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
        except:
            font = ImageFont.load_default()
        
        draw.text((x + scale_px // 2 - 10, y + 10), "1m", fill=(0, 50, 140), font=font)

    def add_orientation(self, draw, x, y):
        """Agrega orientación XY (X arriba, Y izquierda)"""
        arrow_len = 30
        
        # Flecha X (arriba)
        draw.line([(x, y), (x, y - arrow_len)], fill=(0, 50, 140), width=2)
        draw.polygon([(x, y - arrow_len), (x - 5, y - arrow_len + 10), (x + 5, y - arrow_len + 10)],
                    fill=(0, 50, 140))
        
        # Flecha Y (izquierda)
        draw.line([(x, y), (x - arrow_len, y)], fill=(0, 50, 140), width=2)
        draw.polygon([(x - arrow_len, y), (x - arrow_len + 10, y - 5), (x - arrow_len + 10, y + 5)],
                    fill=(0, 50, 140))
        
        # Etiquetas
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 14)
        except:
            font = ImageFont.load_default()
        
        draw.text((x + 5, y - arrow_len - 20), "X", fill=(0, 50, 140), font=font)
        draw.text((x - arrow_len - 20, y + 10), "Y", fill=(0, 50, 140), font=font)

    def add_header(self, draw, canvas_width, canvas_height):
        """Agrega nombre del archivo en esquina superior"""
        timestamp = datetime.now().strftime('%H-%M-%S')
        filename = f'{self.team_name}-{self.mission_name}-{timestamp}'
        
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 14)
        except:
            font = ImageFont.load_default()
        
        draw.text((20, 20), filename, fill=(0, 44, 207), font=font)

    def add_robot_path(self, canvas, trajectory, x_offset, y_offset):
        """Dibuja camino del robot en magenta"""
        draw = ImageDraw.Draw(canvas)
        
        # Convertir de metros a pixels (0.05m/px)
        trajectory_px = [(x_offset + x / self.resolution, 
                         y_offset + y / self.resolution) for x, y in trajectory]
        
        if len(trajectory_px) > 1:
            draw.line(trajectory_px, fill=(120, 0, 140), width=2)  # Magenta

    def add_detected_objects(self, canvas, objects, x_offset, y_offset):
        """Dibuja objetos detectados"""
        draw = ImageDraw.Draw(canvas)
        
        for obj_type, positions in objects.items():
            if obj_type == 'apriltag':
                color = (255, 200, 0)  # Amarillo
                symbol = '#'
            elif obj_type == 'hazmat':
                color = (255, 100, 30)  # Naranja
                symbol = '@@'
            elif obj_type == 'object':
                color = (240, 10, 10)   # Rojo
                symbol = '@@'
            else:
                continue
            
            for x, y, label in positions:
                # Convertir a pixels
                px = x_offset + x / self.resolution
                py = y_offset + y / self.resolution
                
                # Dibujar diamante de 30cm
                diamond_size = int(0.30 / (2 * self.resolution))  # 3 pixels
                draw.polygon([
                    (px, py - diamond_size),
                    (px + diamond_size, py),
                    (px, py + diamond_size),
                    (px - diamond_size, py)
                ], fill=color)

def main():
    """Ejemplo de uso"""
    import sys
    
    if len(sys.argv) > 1:
        pgm_file = sys.argv[1]
    else:
        pgm_file = '/home/testrobotica/RRL-2026-Robot/maps/mapa_final.pgm'
    
    if not os.path.exists(pgm_file):
        print(f"Error: {pgm_file} no existe")
        return
    
    generator = GeoTIFFGenerator(
        pgm_file,
        output_dir='/home/testrobotica/RRL-2026-Robot/maps',
        team_name='YourTeamName',
        mission_name='Prelim1'
    )
    
    # Generar sin objetos detectados por ahora
    generator.create_geotiff()

if __name__ == '__main__':
    main()