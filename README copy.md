# RoboticaWS - VESC Control System

Workspace de ROS 2 para control de motores VESC en sistemas robóticos.

## 🚀 Inicio Rápido

Para correr este proyecto, necesitas compilarlo usando:

```bash
colcon build
source install/setup.bash
```

## Descripción

Este proyecto implementa una interfaz en C++ para comunicación con controladores VESC (Vedder Electronic Speed Controller) a través de puerto serial en ROS 2.

## Características

- ✅ Comunicación serial con motores VESC
- ✅ Manejo de paquetes con CRC16
- ✅ Lectura de telemetría (corriente, temperatura, RPM)
- ✅ Smart pointers para gestión segura de memoria
- ✅ Logging integrado con ROS 2

## Estructura del Proyecto

```
roboticaWS/
├── src/
│   ├── control_pkg/         # Paquete de control
│   └── robot_pkg/           # Paquete principal
│       ├── include/
│       │   └── robot_pkg/
│       │       ├── VESC.hpp # Interfaz VESC
│       │       └── arm.hpp  # Control del brazo
│       └── src/
│           ├── VESC.cpp     # Implementación VESC
│           └── arm.cpp      # Implementación brazo
├── build/                   # Archivos de compilación
├── install/                 # Instalación
└── log/                     # Logs
```

## Requisitos

- ROS 2 (Humble/Foxy)
- C++17 o superior
- `serial` library (`ros-Humble-serial`)
- `rclcpp`

## Instalación

```bash
# Clonar el repositorio
git clone https://github.com/tu-usuario/roboticaWS.git
cd roboticaWS

# Instalar dependencias
sudo apt update
sudo apt install ros-<distro>-serial

# Compilar
colcon build

# Source
source install/setup.bash
```

## Uso

### Ejemplo básico: Control de un motor VESC

```cpp
#include "robot_pkg/VESC.hpp"

int main() {
    // Crear conexión VESC
    VESC motor("/dev/ttyUSB0", 1, 115200, 1000);
    
    // Conectar
    if (motor.connect()) {
        // Leer telemetría
        auto data = motor.get_values();
        
        // Desconectar
        motor.disconnect();
    }
    
    return 0;
}
```

### Control de brazo robótico

```cpp
#include "robot_pkg/arm.hpp"

int main() {
    Arm brazo;
    
    if (brazo.initialize()) {
        brazo.setMotorSpeed(0, 1000.0);  // Motor 0 a 1000 RPM
    }
    
    return 0;
}
```

## API VESC

### Constructor
```cpp
VESC(std::string port, uint8_t id, int baud = 115200, int to = 1000);
```

### Métodos principales
- `bool connect()` - Conectar al VESC
- `void disconnect()` - Desconectar del VESC

### Métodos estáticos
- `uint16_t crc16(data)` - Calcular CRC16
- `std::vector<uint8_t> find_packet(response)` - Extraer paquete válido
- `float current_motor(data)` - Obtener corriente del motor
- `float temp_mos1(data)` - Obtener temperatura FET
- `float temp_motor(data)` - Obtener temperatura del motor

## Configuración de puertos seriales

```bash
# Dar permisos al puerto serial
sudo chmod 666 /dev/ttyUSB0

# O agregar usuario al grupo dialout (permanente)
sudo usermod -a -G dialout $USER
# Luego reiniciar sesión
```

## Compilación

```bash
cd ~/roboticaWS
colcon build --packages-select robot_pkg
source install/setup.bash
```

## Troubleshooting

### Error: "Permission denied" en puerto serial
```bash
sudo chmod 666 /dev/ttyUSB0
```

### Error: "No module named serial"
```bash
sudo apt install ros-<distro>-serial
```

## Contribuir

1. Fork el proyecto
2. Crea una rama (`git checkout -b feature/nueva-funcionalidad`)
3. Commit tus cambios (`git commit -am 'Agregar nueva funcionalidad'`)
4. Push a la rama (`git push origin feature/nueva-funcionalidad`)
5. Abre un Pull Request

## Licencia

Este proyecto está bajo la Licencia MIT. Ver el archivo `LICENSE` para más detalles.

## Autores

- Tu Nombre - *Trabajo inicial*

## Agradecimientos

- Comunidad ROS 2
- VESC Project (Benjamin Vedder)
