
# 🤖 ROS 2 Mobile Manipulator Architecture

> **Sistema de control modular para robot móvil con brazo manipulador (8 DOF) con Abstracción de Hardware y Autodetección de Puertos.**

![ROS2](https://img.shields.io/badge/ROS_2-Humble%2FIron-22314E?style=for-the-badge&logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![Platform](https://img.shields.io/badge/Platform-Linux%20%2F%20Jetson-green?style=for-the-badge)

## 📖 Descripción General

Este proyecto implementa una arquitectura de software robusta en **ROS 2 (C++)** para controlar un robot de rescate/exploración compuesto por un chasis tipo tanque (con flippers) y un brazo manipulador.

La característica principal es su **Capa de Abstracción de Hardware (HAL)**, que desacopla la lógica de control de la configuración física. El sistema utiliza un mecanismo de **Autodetect ID** que escanea los puertos seriales, identifica cada motor por su firmware ID único y reasigna los objetos de control dinámicamente según un archivo JSON.

## 🚀 Características Clave

* **Arquitectura de Nodos Separados:**
    * `control_publisher`: Interfaz de usuario (Joystick/Teclado).
    * `robot_listener`: Orquestador de hardware y lógica de movimiento.
* **Hardware Autodetect System:** Olvídate de configurar `/dev/ttyUSB0` manualmente. El sistema busca los motores por su ID lógico.
* **Máquina de Estados de Seguridad:**
    * **Modo Navegación:** Controla orugas y flippers. El brazo se bloquea mecánicamente (brake).
    * **Modo Manipulación:** Controla las articulaciones del brazo. El chasis se frena totalmente.
* **Cálculo de Motores Independientes:** Gestión de 8 hilos/drivers independientes con cálculo de cinemática local: `Input * MaxRPM * Orientación`.

---

## 🏗️ Arquitectura del Sistema

### Estructura de Paquetes
```text
src/
├── control_publisher/       # [Nodo] Lee Joystick -> Publica /joy_control
└── robot_listener/          # [Nodo] Lógica Principal
    ├── config/
    │   └── motor_config.json  # "La Verdad Absoluta" del hardware
    ├── include/robot_listener/
    │   ├── hardware_manager.hpp # Clase encargada del escaneo de puertos
    │   ├── motor_driver.hpp     # Driver genérico (Serial comms)
    │   ├── body_controller.hpp  # Cinemática diferencial (Tanque)
    │   └── arm_controller.hpp   # Cinemática del brazo
    └── src/
        ├── main.cpp             # Máquina de estados
        ├── hardware_manager.cpp
        └── ...

```

### Flujo de Inicialización (Startup)

1. **Lectura de Config:** Se carga `motor_config.json`.
2. **Escaneo:** `HardwareManager` itera sobre `/dev/ttyUSB*`.
3. **Handshake:** Se envía ping a cada puerto. El dispositivo responde con su ID único (ej. `0xA1`).
4. **Mapeo:** El sistema vincula el puerto físico con el objeto lógico (ej. ID `0xA1` -> `Left_Track`).
5. **Inyección:** Los objetos `Motor` inicializados se pasan a las clases `Body` y `Arm`.

---

## ⚙️ Configuración (JSON)

Para cambiar la configuración física del robot (ej. invertir un motor o limitar su velocidad), edita `config/motor_config.json`. **No es necesario recompilar.**

```json
{
  "motors": [
    {
      "logical_name": "left_track",
      "hardware_id": 10,
      "max_rpm": 255,
      "orientation": 1
    },
    {
      "logical_name": "right_track",
      "hardware_id": 11,
      "max_rpm": 255,
      "orientation": -1
    },
    {
      "logical_name": "arm_base",
      "hardware_id": 20,
      "max_rpm": 60,
      "orientation": 1
    }
    // ... hasta completar los 8 motores
  ]
}

```

---

## 🎮 Modos de Operación

El `main.cpp` actúa como un switch que redirige los comandos del joystick.

| Modo | Indicador | Comportamiento |
| --- | --- | --- |
| **DRIVE + FLIPPERS** | LED Verde (Físico) | Joystick Izq: Avance/Retroceso<br>

<br>Joystick Der: Giro<br>

<br>Botones: Subir/Bajar Flippers |
| **ARM CONTROL** | LED Azul (Físico) | Joystick Izq: Base/Hombro<br>

<br>Joystick Der: Codo/Muñeca<br>

<br>Botones: Gripper |

> **Nota de Seguridad:** Al cambiar de modo, el sistema envía automáticamente un comando de `velocidad 0` a los motores del modo anterior antes de cambiar el control.

---

## 🛠️ Instalación y Build

### Prerrequisitos

* ROS 2 (Humble, Iron o Jazzy)
* Librería Serial (ej. `serial` o `libserial-dev`)
* Librería JSON (ej. `nlohmann-json3-dev`)

### Pasos

1. **Clonar el repositorio:**
```bash
cd ~/ros2_ws/src
git clone [https://github.com/atzin-cruz/ros2_mobile_manipulator.git](https://github.com/atzin-cruz/ros2_mobile_manipulator.git)

```


2. **Instalar dependencias:**
```bash
sudo apt install nlohmann-json3-dev
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

```


3. **Compilar:**
```bash
colcon build --symlink-install
source install/setup.bash

```



## ▶️ Ejecución

### 1. Iniciar el Robot (Listener)

Este nodo iniciará el proceso de autodetect. Asegúrate de que los USB estén conectados y los drivers alimentados.

```bash
ros2 run robot_listener listener_node

```

*Busca en la terminal mensajes como:* `[INFO] Motor 'left_track' paired with /dev/ttyUSB1`.

### 2. Iniciar el Control (Publisher)

```bash
ros2 run control_publisher joy_publisher

```

---

