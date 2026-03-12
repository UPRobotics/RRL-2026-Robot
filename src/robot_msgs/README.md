# robot_msgs

Paquete de mensajes ROS 2 personalizados para la comunicación entre los nodos del robot y la estación base.

---

## Mensajes

### `MotorTelemetry.msg`
Datos **leídos en tiempo real del hardware** (VESC → Robot → Estación).

| Campo | Tipo | Descripción |
|---|---|---|
| `motor_id` | `uint8` | ID del controlador VESC reportado por el firmware |
| `motor_name` | `string` | Nombre del motor (ej. "Track Izquierdo") |
| `rpm` | `int32` | RPM actuales medidos por el VESC |
| `duty_cycle` | `float32` | Duty cycle actual (0.0 a 1.0) |
| `voltage` | `float32` | Voltaje del bus de entrada (Volts) |
| `control_mode` | `uint8` | Modo activo: `0` = RPM, `1` = duty cycle |
| `inverted` | `bool` | Orientación eléctrica del motor |

---

### `MotorConfig.msg`
Parámetros **enviados por el operador** para modificar el comportamiento del robot (Estación → Robot).

| Campo | Tipo | Descripción |
|---|---|---|
| `config_index` | `uint8` | Índice del motor en el arreglo `motors[]` del `config.json` |
| `motor_vesc_id` | `uint8` | ID del controlador VESC (firmware CAN/UART), usado por `autoConnect()` |
| `motor_name` | `string` | Nombre del motor (para display y validación) |
| `rpm_limit` | `float32` | RPM máximos permitidos |
| `duty_cycle_limit` | `float32` | Duty cycle máximo permitido (0.0 a 1.0) |
| `control_mode` | `uint8` | Modo deseado: `0` = RPM, `1` = duty cycle |
| `inverted` | `bool` | Orientación eléctrica del motor |

---

## ¿Por qué dos mensajes distintos?

Son dos mensajes porque hacen cosas **opuestas en direcciones opuestas**:

```
VESC ──► arm_node/body_node ──► MotorTelemetry ──► telemetry_node ──► JSON agregado ──► telemetry_ui
                                  (lo que ES)

telemetry_ui ──► MotorConfig ──► telemetry_node ──► MotorConfig ──► arm_node/body_node ──► config.json
                                  (lo que DEBE SER)
```

`MotorTelemetry` son **lecturas de sensor**: el robot reporta lo que el hardware está midiendo en ese instante. Son de solo lectura y se publican cada 50 ms automáticamente.

`MotorConfig` son **comandos de configuración**: el operador decide cambiar un límite de RPM o de duty cycle desde la UI, y el robot lo aplica en RAM y lo persiste en disco.

Si fueran el mismo mensaje, cualquier publicación de telemetría podría sobrescribir accidentalmente los parámetros de configuración, o un cambio de config podría interpretarse como una lectura del sensor.

---

## Pipeline completo

### Telemetría (Robot → Estación → UI)

```
VESC (serial)
    │  get_telemetry() cada 50ms
    ▼
arm_node / body_node
    │  publica robot_msgs/MotorTelemetry
    │  (solo los motores conectados publican)
    ▼
/arm_hip/telemetry
/arm_shoulder/telemetry
/arm_elbow/telemetry
/arm_roll/telemetry
/arm_pitch/telemetry
/arm_claw/telemetry
/body_left/telemetry
/body_right/telemetry
/body_left_flipper/telemetry
/body_right_flipper/telemetry
    │
    ▼
telemetry_node  (estación)
    │  agrega SOLO los motores que han enviado datos
    │  serializa a JSON con todos los campos + config_index
    │  publica std_msgs/String cada 20ms
    ▼
/telemetry/aggregated   (QoS: BEST_EFFORT, KeepLast 5)
    │
    ▼
telemetry_ui_node  (estación)
    │  parsea JSON
    │  muestra grid dinámico (solo motores activos)
    │  sidebar con voltaje y potencia estimada
    └── panel de edición de configuración por motor
```

**Formato JSON de `/telemetry/aggregated`:**

```json
{
  "timestamp_ms": 1710000000000,
  "motors": [
    {
      "config_index": 1,
      "motor_id": 1,
      "motor_name": "Track Izquierdo",
      "rpm": 1500,
      "duty_cycle": 0.45,
      "voltage": 24.1,
      "control_mode": 0,
      "inverted": false
    }
  ]
}
```

> El arreglo `motors` solo contiene los motores que han enviado telemetría.
> Si solo hay 2 motores conectados, el JSON contiene 2 entradas y la UI muestra 2 tarjetas.

---

### Configuración (UI → Estación → Robot → Disco)

```
telemetry_ui_node  (panel de edición en la UI)
    │  operador ajusta rpm_limit, duty_cycle_limit, control_mode, inverted
    │  publica robot_msgs/MotorConfig
    ▼
/ground_station/motor_config   (QoS: RELIABLE, KeepLast 10)
    │
    ▼
telemetry_node  (estación)
    │  re-publica sin modificar
    ▼
/robot_config/update   (QoS: RELIABLE, KeepLast 10)
    │
    ├──► arm_node   (índices 4–9: Cadera, Hombro, Codo, Roll, Pitch, Grip)
    └──► body_node  (índices 0–3: Flipper Trasero, Track Izq, Track Der, Flipper Delantero)
              │
              ├── actualiza MotorSettings en RAM (protegido por mutex)
              └── escribe robot_pkg/config/config.json en disco
```

---

### `config_index` — mapeo de motores

| `config_index` | Motor | Nodo |
|---|---|---|
| 0 | Flipper Trasero | `body_node` |
| 1 | Track Izquierdo | `body_node` |
| 2 | Track Derecho | `body_node` |
| 3 | Flipper Delantero | `body_node` |
| 4 | Cadera | `arm_node` |
| 5 | Hombro | `arm_node` |
| 6 | Codo | `arm_node` |
| 7 | Roll | `arm_node` |
| 8 | Pitch | `arm_node` |
| 9 | Grip | `arm_node` |

Cada nodo ignora silenciosamente (`default: return`) los mensajes con índices que no le corresponden.

> **Nota:** El sistema es dinámico — no todos los motores necesitan estar conectados.
> `telemetry_node` solo agrega los motores que han enviado datos, y `telemetry_ui_node`
> adapta su grid automáticamente al número de motores activos.

---

## QoS

| Flujo | QoS | Razón |
|---|---|---|
| Telemetría (`MotorTelemetry`, `/telemetry/aggregated`) | BEST_EFFORT, KeepLast(5) | Minimiza congestión WiFi; datos recientes importan más que completitud |
| Configuración (`MotorConfig`) | RELIABLE, KeepLast(10) | Los cambios de config deben llegar; no son tiempo real |

---

## Ejemplo de uso

Cambiar el límite de RPM del Track Izquierdo (índice 1) a 3000:

```bash
ros2 topic pub --once /ground_station/motor_config robot_msgs/msg/MotorConfig \
  '{config_index: 1, motor_vesc_id: 1, motor_name: "Track Izquierdo", rpm_limit: 3000.0, duty_cycle_limit: 1.0, control_mode: 0, inverted: false}'
```

El cambio se aplica inmediatamente en `body_node` y se guarda en `config.json` para que persista al siguiente arranque.

Verificar telemetría agregada:

```bash
ros2 topic echo /telemetry/aggregated
```
