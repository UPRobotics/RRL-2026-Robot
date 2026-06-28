# 🤖 RoboCup Rescue 2026 - Autonomy Dashboard

**Dashboard profesional para visualización y control de navegación autónoma en competencias de RoboCup Rescue.**

## 📋 Características

### ✨ Visualización en Tiempo Real
- **Mapa 2D SLAM** - Muestra el layout del laberinto con posición del robot en tiempo real
- **Nube 3D** - Visualización de puntos Fast-LIO con escalado de altura (colores por Z)
- **Pose del Robot** - Coordenadas X, Y y ángulo theta actualizado en vivo
- **Exploración** - Barra de progreso mostrando % de área explorada

### 🎮 Controles
- ▶️ **START MAPPING** - Inicia captura de mapas
- ⏸ **PAUSE** - Pausa el mapeo
- ▶️ **RESUME** - Continúa mapeo
- 💾 **QUICK MAP** - Guarda mapa rápido (<5 min)
- 💾 **COMPREHENSIVE MAP** - Guarda mapa completo (>15 min)
- 📦 **SAVE 3D CLOUD** - Exporta como PLY
- 📋 **SAVE OBJECTS** - Exporta POIs como CSV
- 🗺️ **EXPORT GeoTIFF** - Genera GeoTIFF con información espacial
- ⚙️ **AUTO-EXPORT** - Exporta automáticamente a los 20 minutos
- 🔄 **RESET MISSION** - Reinicia la misión

### 📊 Estado del Sistema
- Modo de autonomía (IDLE, MAPPING, EXPLORING, NAVIGATING, FAULT)
- Conexión ROS (Conectado/Desconectado)
- Posición actual del robot (X, Y, Θ)
- Área explorada (%)
- Número de puntos nube (cloud points)
- Tiempo transcurrido / Tiempo máximo
- Objetos detectados
- Información del equipo y misión

### 🔄 Arquitectura

```
┌─────────────────────────────────────────────┐
│   autonomy_dashboard_node.py                │
│   (ROS Node - Agrega datos)                 │
└────────┬────────────────────────────────────┘
         │
         ├── Subscriptions:
         │   ├── /cloud_registered (PointCloud2)
         │   ├── /map (OccupancyGrid)
         │   ├── /odom (Odometry)
         │   ├── /amcl_pose (PoseStamped)
         │   └── /autonomy/status (String)
         │
         └── Publications:
             ├── /autonomy/status
             ├── /autonomy/robot_pose
             ├── /autonomy/mapped_area_percent
             └── /autonomy/detected_objects

┌─────────────────────────────────────────────┐
│   autonomy_ui.py (PyQt5)                    │
│   ┌───────────────┬────────────────────┐   │
│   │   Map 2D      │  Map 3D Cloud      │   │
│   ├───────────────┴────────────────────┤   │
│   │   Status Panel (Pose, Area, etc)   │   │
│   ├────────────────────────────────────┤   │
│   │   Control Buttons (Start/Save/etc) │   │
│   └────────────────────────────────────┘   │
└─────────────────────────────────────────────┘
```

## 📦 Instalación

### Requisitos
```bash
sudo apt install python3-pyqt5 python3-matplotlib
pip3 install numpy
```

### Compilar Paquete
```bash
cd ~/RRL-2026-Robot
colcon build --packages-select autonomy_dashboard_pkg
source install/setup.bash
```

## 🚀 Uso

### Opción 1: Launch file completo
```bash
ros2 launch autonomy_dashboard_pkg autonomy_dashboard.launch.py \
  team_name:=YourTeam \
  mission:=Prelim1
```

### Opción 2: Ejecutar nodo ROS
```bash
ros2 run autonomy_dashboard_pkg autonomy_dashboard_node.py
```

### Opción 3: Ejecutar solo interfaz PyQt5
```bash
ros2 run autonomy_dashboard_pkg autonomy_ui.py
```

## 📁 Estructura del Proyecto

```
autonomy_dashboard_pkg/
├── autonomy_dashboard/
│   ├── __init__.py
│   ├── autonomy_dashboard_node.py    # Nodo ROS principal
│   ├── autonomy_ui.py                # Interfaz PyQt5
│   └── widgets/
│       ├── __init__.py
│       ├── map_viewer_2d.py          # Widget mapa 2D
│       ├── cloud_viewer_3d.py        # Widget nube 3D
│       └── status_panel.py           # Panel de estado
├── config/
│   └── autonomy_params.yaml          # Parámetros
├── launch/
│   └── autonomy_dashboard.launch.py  # Launch file
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 🎯 Requisitos RoboCup 2026

El dashboard cumple con los requisitos de Mapping & Object Detection:

### ✅ Formatos Soportados
- **2D Map**: GeoTIFF (formato especificado)
- **3D Map**: PLY (point cloud)
- **Objects**: CSV con formato RoboCup 2026

### ✅ Funcionalidades Autonomía
- Mapeo autónomo del laberinto
- Detección automática de objetos
- Auto-exportación a los 20 minutos
- Información de pose en tiempo real
- Visualización del progreso de exploración

### ✅ Multipliers
- **Teleop (1x)**: Operador controla manualmente
- **Comms Degradation (2x)**: Con comunicación degradada
- **Autonomy (5x)**: Navegación totalmente autónoma

## 🔧 Configuración

Editar `config/autonomy_params.yaml`:

```yaml
autonomy_dashboard_node:
  ros__parameters:
    team_name: "TuEquipo"
    mission: "Prelim1"
    output_dir: "/ruta/salida"
    max_mapping_time: 1200  # 20 min
```

## 🎬 Flujo de Uso en Competencia

1. **Conecta Hardware**
   - LIDAR (Livox/Ouster)
   - Robot base ROS2
   - Sensor de objetos

2. **Inicia ROS Stack**
   ```bash
   # Terminal 1: Fast-LIO + Nav2
   ros2 launch fastlio_pkg mapping.launch.py
   
   # Terminal 2: Autonomy Dashboard
   ros2 launch autonomy_dashboard_pkg autonomy_dashboard.launch.py
   ```

3. **En el Dashboard**
   - Presiona "START MAPPING"
   - Navega el robot por el laberinto (automático o teleop)
   - Presiona "ENABLE AUTO-EXPORT" (se guardará en 20 min)
   - O manualmente: presiona "SAVE" en cualquier momento

4. **Exporta Resultados**
   - Los archivos se guardan en formato RoboCup 2026
   - PLY para nube 3D
   - GeoTIFF para mapa 2D
   - CSV con objetos detectados

## 📊 Datos Exportados

### Archivos PLY
```
RoboCup2026-TeamName-Mission-HH-MM-SS-map.ply
```
- Formato ASCII PLY
- X, Y, Z de cada punto
- Compatible con CloudCompare, Meshlab

### GeoTIFF
```
TeamName-Mission-HH-MM-SS.tiff
```
- Mapa rasterizado con grid
- Escala y orientación incluidas
- Compatible con QGIS, ArcGIS

### CSV Objects
```
RoboCup2026-TeamName-Mission-HH-MM-SS-pois.csv
```
- Formato específico RoboCup 2026
- detection,time,type,name,x,y,z,robot,mode
- Objetos detectados (QR, Hazmat, Víctimas)

## 🐛 Troubleshooting

### "ImportError: No module named 'PyQt5'"
```bash
sudo apt install python3-pyqt5
```

### "No se reciben datos del cloud"
- Verificar que Fast-LIO está publicando en `/cloud_registered`
- Revisar: `ros2 topic list` y `ros2 topic info /cloud_registered`

### "No se ve el mapa SLAM"
- Verificar que Nav2/SLAM está publicando en `/map`
- Revisar: `ros2 topic echo /map` (primeras líneas)

### Interfaz lenta
- Reducir `max_points` en config (downsampling)
- Aumentar update_rate: `update_rate: 5` (Hz)

## 📝 Licencia

BSD License - RoboCup Rescue 2026

## 👥 Autor

RoboCup Rescue Team 2026

## 🔗 Referencias

- RoboCup Rescue Rules 2026D
- Fast-LIO2 SLAM
- Nav2 Framework
- PyQt5 Documentation
