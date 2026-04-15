# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

ROS 2 Humble workspace for a mobile rescue robot. Runs on Ubuntu 22.04 (x86_64)
and NVIDIA Jetson Orin (aarch64). All C++ packages use `ament_cmake`; magnetometer
is pure Python (`ament_python`).

---

## Package Map

| Package | Lang | What it does |
|---------|------|--------------|
| `robot_msgs` | IDL | Custom message types — see Message Definitions below |
| `robot_pkg` | C++ | VESC motor control — body (4 motors) + arm (6 motors) |
| `control_pkg` | C++ | Reads `/joy`, publishes `ControlInput` + mode toggle; runs on ground station |
| `camera_pkg` | C++ | Multi-RTSP camera viewer — **desktop only, skip on Jetson** |
| `detections_pkg` | C++ | QR / motion / hazmat detection — **desktop only, skip on Jetson** |
| `thermal_pkg` | C++ | MLX90640 thermal camera reader + display |
| `mic_pkg` | C++ | USB mic capture + Vosk speech-to-text |
| `magnetometer_pkg` | Python | Serial magnetometer reader + Tkinter/Matplotlib GUI |
| `telemetry_pkg` | C++ | Aggregates all motor telemetry, relays config |
| `telemetry_ui_pkg` | C++ | SDL2 ground-station dashboard |

---

## Build

```bash
# Full build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Jetson (skip desktop-only packages)
colcon build --symlink-install \
  --packages-ignore camera_pkg detections_pkg

# Single package
colcon build --packages-select robot_pkg

# Force clean rebuild of a package
rm -rf build/robot_pkg install/robot_pkg
colcon build --packages-select robot_pkg
```

Convenience scripts in `bash_files/`:
- `compile_computer.sh` — builds `robot_msgs`, `telemetry_ui_pkg`, `control_pkg`
- `compile_jetson.sh` — clean-builds `robot_msgs`, `robot_pkg`, `telemetry_pkg`
- `execute_computer.sh` — launches `control_pkg joystick.launch.py` (ground station)
- `execute_robot.sh` — launches `robot_pkg robot.launch.py` (robot side)

## Running Nodes

```bash
# Ground station (operator side)
ros2 launch control_pkg joystick.launch.py     # joy_node + joystick_node
ros2 run telemetry_ui_pkg telemetry_ui_node    # SDL2 dashboard

# Robot side
ros2 launch robot_pkg robot.launch.py          # arm_node + body_node + telemetry_node

# Individual nodes
ros2 run robot_pkg body_node
ros2 run robot_pkg arm_node
ros2 launch thermal_pkg thermal_camera.launch.py serial_port:=/dev/ttyUSB0
ros2 launch magnetometer_pkg magnetometer.launch.py

# FastDDS unicast (required over WiFi — run before launching anything)
source fastdds/setup.sh    # edit fastdds/fastdds_wifi.xml first with ROBOT_IP / STATION_IP
```

---

## Topic / QoS Reference

### Control pipeline
```
joy_node (ros-humble-joy)
  └─ /joy  (sensor_msgs/Joy)
       └─ joystick_node (control_pkg)
            ├─ /control/input   robot_msgs/ControlInput  BEST_EFFORT KeepLast(1)
            └─ /robot/mode      std_msgs/UInt8           BEST_EFFORT KeepLast(1)
                 └─ body_node / arm_node (robot_pkg)
```

Mode toggle: Xbox **B button** (buttons[1]) — rising-edge, toggles between 0=MOVEMENT and 1=ARM.

Joystick axis mapping (sensor_msgs/Joy.axes[]):
```
axes[0] → left_x    (rotation / arm)
axes[1] → left_y    (forward/back / arm)
axes[2] → trigger_L (normalized: 0=released, 1=full)
axes[3] → right_x   (rear flipper / arm roll)
axes[4] → right_y   (front flipper / arm elbow)
axes[5] → trigger_R (normalized: 0=released, 1=full)
axes[6] → dpad_x    (-1=left, +1=right)
axes[7] → dpad_y    (+1=up, -1=down)
```

### Motor telemetry (robot_msgs/MotorTelemetry, BEST_EFFORT KeepLast(5))
```
/body_left/telemetry          /arm_hip/telemetry
/body_right/telemetry         /arm_shoulder/telemetry
/body_left_flipper/telemetry  /arm_elbow/telemetry
/body_right_flipper/telemetry /arm_roll/telemetry
                               /arm_pitch/telemetry
                               /arm_claw/telemetry
```

### Config update pipeline (RELIABLE KeepLast(10))
```
/ground_station/motor_config  (robot_msgs/MotorConfig)
  └─ telemetry_node (telemetry_pkg)
       └─ /robot_config/update  (robot_msgs/MotorConfig)
            └─ body_node / arm_node (robot_pkg)
```

### Other topics
```
/thermal_data              Float32MultiArray  768 floats (32×24)   thermal_pkg → camera_pkg
/telemetry/aggregated      String (JSON, 50 Hz)                     telemetry_pkg
/telemetryJSON/arm_max_rpm Float32                                   telemetry_pkg
```

---

## Message Definitions

**robot_msgs/ControlInput** — single message replacing the 4 separate Float32 topics:
```
uint8   mode            # 0=MOVEMENT, 1=ARM
float32 left_x
float32 left_y
float32 right_x
float32 right_y
float32 trigger_left    # normalized 0–1
float32 trigger_right
float32 dpad_x          # -1=left, +1=right
float32 dpad_y          # +1=up, -1=down
bool    bumper_left     # L1
bool    bumper_right    # R1
```

**robot_msgs/DPadConfig** — sent by telemetry_ui to adjust motor limits via D-pad:
```
float32 rpm_limit
float32 duty_cycle_limit
```

**robot_msgs/JoystickAxes** — (defined but unused; ControlInput is preferred):
```
float32 left_x / left_y / right_x / right_y
```

**robot_msgs/MotorTelemetry:**
```
uint8   motor_id        # VESC firmware ID
string  motor_name
int32   rpm
float32 duty_cycle      # 0.0–1.0
float32 voltage         # bus voltage in V
uint8   control_mode    # 0=RPM, 1=duty_cycle
bool    inverted
```

**robot_msgs/MotorConfig:**
```
uint8   config_index       # index in motors[] (0–9)
uint8   motor_vesc_id
string  motor_name
float32 rpm_limit
float32 duty_cycle_limit   # 0.0–1.0
uint8   control_mode       # 0=RPM, 1=duty_cycle
bool    inverted
```

---

## Motor Layout

### Body (body_node, ttyACM0–3)
| config_index | Motor name | VESC ID | ttyACM | Notes |
|---|---|---|---|---|
| 0 | Flipper Trasero | 2 | ACM2 | inverted=true |
| 1 | Track Izquierdo | 0 | ACM0 | |
| 2 | Track Derecho | 1 | ACM1 | inverted=true |
| 3 | Flipper Delantero | 3 | ACM3 | |

Tank drive: `left = Y + X`, `right = Y - X`
Front flipper ← right_y, Rear flipper ← right_x

**Motor groups** (used in telemetry_ui for bulk config):
- `Flippers`: config_index {0, 3}
- `Tracks`: config_index {1, 2}

### Arm (arm_node, ttyACM4)
| config_index | Motor name | VESC ID | mode | rpm_limit |
|---|---|---|---|---|
| 4 | Hip | 4 | RPM | 3000 |
| 5 | Shoulder | 5 | duty_cycle | 3000 |
| 6 | Elbow | 6 | duty_cycle | 3000 |
| 7 | Roll | 7 | RPM | 2000 |
| 8 | Pitch | 8 | duty_cycle | 1000 |
| 9 | Grip | 9 | duty_cycle | 1000 |

All arm `duty_cycle_limit = 0.1`

---

## VESC Serial Protocol

Library: `LibSerial` (libserial-dev)
Baudrate: 115200, timeout: 1000 ms
Frame: `[0x02][length][payload][CRC_H][CRC_L][0x03]`

Key packet IDs:
- `0x04` = COMM_GET_VALUES (request telemetry)
- `0x05` = COMM_SET_DUTY (duty cycle × 100000 as big-endian int32)
- `0x08` = COMM_SET_RPM (big-endian int32)

Standalone VESC test tool (no ROS required): `src/robot_pkg/src/pytest/pytest.py`
Edit `SERIAL_PORTS` dict at the top to target specific ports.

---

## Config File (runtime motor config)

Loaded by body_node and arm_node from:
`ament_index_cpp::get_package_share_directory("robot_pkg") + "/config/config.json"`

Updated at runtime via `/robot_config/update` topic.

---

## Multithreading Pattern (robot_pkg)

Each motor drive loop runs in its own callback group → parallel execution in
`MultiThreadedExecutor`. Joystick values shared between threads via `std::atomic<float>`.

```cpp
std::atomic<float> left_y_{0.0f};
rclcpp::CallbackGroup::SharedPtr cg_left_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
```

Telemetry timer: 50 ms (20 Hz). Drive timers: 10 ms (100 Hz) per motor.

---

## Docker

Image base: `osrf/ros:humble-desktop-full`, platform `linux/arm64`

```bash
docker compose build
docker compose up <service>

# On actual robot hardware
docker compose -f docker-compose.yml -f docker-compose.linux.yml up <service>

# Incremental C++ rebuild inside container
docker compose -f docker-compose.yml -f docker-compose.dev.yml run --rm dev \
  bash -c "colcon build --packages-select robot_pkg"
```

Services: `robot`, `body`, `arm`, `sim`, `joystick`, `camera`, `detections`,
`thermal`, `mic`, `magnetometer`, `telemetry_ui`

---

## Device Paths (Linux)

| Device | Path | Group |
|--------|------|-------|
| Body VESC motors | /dev/ttyACM0–3 | dialout |
| Arm VESC | /dev/ttyACM4 | dialout |
| Joystick | /dev/input/js0 | input |
| Thermal camera (I2C) | /dev/i2c-1 | i2c |
| Microphone (ALSA) | /dev/snd | audio |
| Magnetometer / thermal serial | /dev/ttyUSB0 | dialout |

Add user to groups: `sudo usermod -aG dialout,input,audio $USER`

---

## WiFi / FastDDS

Default DDS multicast fails on WiFi. Use unicast profile:

```bash
source fastdds/setup.sh   # sets FASTRTPS_DEFAULT_PROFILES_FILE
```

Edit `fastdds/fastdds_wifi.xml` to set ROBOT_IP and STATION_IP before sourcing.
Discovery port: 7412. Lease duration: 10 s.

See `LATENCY_IMPROVEMENTS.md` for WiFi power-save, CycloneDDS, and UDP buffer tuning.

---

## Camera Config

IPs: `192.168.0.200–206` (7 cameras). Cameras 5–7 require auth (`admin:admin`).
Config: `src/camera_pkg/config/settings.json`
Hazmat models: `src/detections_pkg/config/models/hazmat_best.onnx` (YOLOv8),
`hazmat_resnet18.onnx` (ResNet-18 classifier)

---

## Key System Dependencies

| Package | apt name |
|---------|---------|
| Serial comm | libserial-dev |
| JSON | nlohmann-json3-dev |
| Joystick driver | ros-humble-joy |
| SDL2 graphics | libsdl2-dev, libsdl2-ttf-dev |
| FFmpeg (RTSP) | libavcodec-dev, libavformat-dev, libswscale-dev, libavutil-dev |
| Logging | libspdlog-dev |
| Computer vision | libopencv-dev |
| QR detection | libzbar-dev |
| Audio | libasound2-dev |
| Python serial | python3-serial |
| Python GUI | python3-matplotlib, python3-tk |
