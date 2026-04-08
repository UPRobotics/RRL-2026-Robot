# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

ROS 2 Humble workspace for a mobile rescue robot. Runs on Ubuntu 22.04 (x86_64)
and NVIDIA Jetson Orin (aarch64). All C++ packages use `ament_cmake`; magnetometer
is pure Python (`ament_python`).

---

## Installation (first time only)

```bash
# Ubuntu 22.04
chmod +x install_ubuntu.sh && ./install_ubuntu.sh

# Jetson Orin
chmod +x install_jetson_orin.sh && ./install_jetson_orin.sh
```

The scripts install all system dependencies, Vosk speech models, ONNX Runtime (Ubuntu), and add the user to the `dialout`/`input`/`audio` groups. **Log out and back in after running** for group changes to take effect.

To install ROS dependencies after adding new packages:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

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
```

**Build order matters when doing selective builds:** `robot_msgs` must be built before any
package that depends on it. The scripts in `bash_files/` encode the correct order:

```bash
# Ground station (compile_computer.sh)
colcon build --packages-select robot_msgs
colcon build --packages-select telemetry_ui_pkg
colcon build --packages-select control_pkg

# Jetson / robot (compile_jetson.sh)
colcon build --packages-select robot_msgs
colcon build --packages-select robot_pkg
colcon build --packages-select telemetry_pkg
```

---

## Running

Convenience scripts are in `bash_files/` (run them from inside that directory or adjust paths):

```bash
# Ground station — launches joystick node
bash bash_files/execute_computer.sh   # ros2 launch control_pkg joystick.launch.py

# Robot — launches body + arm + telemetry
bash bash_files/execute_robot.sh      # ros2 launch robot_pkg robot.launch.py
```

Key launch files:
```bash
ros2 launch robot_pkg robot.launch.py          # body + arm + telemetry (all-in-one)
ros2 launch robot_pkg body.launch.py           # body/flippers only
ros2 launch robot_pkg arm.launch.py            # arm only
ros2 launch robot_pkg robot_sim.launch.py      # simulation (no hardware required)
ros2 launch control_pkg joystick.launch.py     # joy_node + joystick_node
ros2 launch thermal_pkg thermal_camera.launch.py serial_port:=/dev/ttyUSB0
ros2 launch magnetometer_pkg magnetometer.launch.py
ros2 launch telemetry_ui_pkg telemetry_ui.launch.py
# Desktop only:
ros2 launch camera_pkg camera_pkg.launch.py
ros2 launch detections_pkg detections.launch.py
ros2 launch mic_pkg mic_transmitter.launch.py
```

### VESC standalone test (no ROS)

`src/robot_pkg/src/pytest/pytest.py` — a standalone Python script for directly
testing VESC serial communication. Edit `SERIAL_PORTS` dict at the top, then:

```bash
python3 src/robot_pkg/src/pytest/pytest.py
```

---

## Package Map

| Package | Lang | What it does |
|---------|------|--------------|
| `robot_msgs` | IDL | Custom message types (MotorTelemetry, MotorConfig) |
| `robot_pkg` | C++ | VESC motor control — body (4 motors) + arm (6 motors) + motor_sim |
| `control_pkg` | C++ | Joystick → directional topics |
| `camera_pkg` | C++ | Multi-RTSP camera viewer — **desktop only, skip on Jetson** |
| `detections_pkg` | C++ | QR / motion / hazmat detection — **desktop only, skip on Jetson** |
| `thermal_pkg` | C++ | MLX90640 thermal camera reader + display |
| `mic_pkg` | C++ | USB mic capture + Vosk speech-to-text |
| `magnetometer_pkg` | Python | Serial magnetometer reader + Tkinter/Matplotlib GUI |
| `telemetry_pkg` | C++ | Aggregates all motor telemetry, relays config |
| `telemetry_ui_pkg` | C++ | SDL2 ground-station dashboard |

`motor_sim.cpp` (in `robot_pkg`) provides a simulated body + arm node for testing
the full pipeline without hardware — launched via `robot_sim.launch.py`.

---

## Topic / QoS Reference

### Control pipeline
```
joy_node (ros-humble-joy)
  └─ /joy  (sensor_msgs/Joy)
       └─ joystick_node (control_pkg)
            ├─ /joystick/left_y   Float32  BEST_EFFORT KeepLast(1)
            ├─ /joystick/left_x   Float32  BEST_EFFORT KeepLast(1)
            ├─ /joystick/right_y  Float32  BEST_EFFORT KeepLast(1)
            └─ /joystick/right_x  Float32  BEST_EFFORT KeepLast(1)
```

Joystick axis mapping (sensor_msgs/Joy.axes[]):
```
axes[0] → left_x   (rotation)
axes[1] → left_y   (forward/back)
axes[3] → right_x  (rear flipper)
axes[4] → right_y  (front flipper)
```

### Motor telemetry (robot_msgs/MotorTelemetry, BEST_EFFORT KeepLast(5))
```
/body_left/telemetry
/body_right/telemetry
/body_left_flipper/telemetry
/body_right_flipper/telemetry
/arm_hip/telemetry
/arm_shoulder/telemetry
/arm_elbow/telemetry
/arm_roll/telemetry
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
/thermal_data               Float32MultiArray  768 floats (32×24)  thermal_pkg → camera_pkg
/telemetry/aggregated       String (JSON)                           telemetry_pkg @ 50 Hz
/telemetryJSON/arm_max_rpm  Float32                                 telemetry_pkg
magnetometer_data           String (JSON)                           magnetometer_pkg internal
```

---

## Message Definitions

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

VESC IDs below reflect the current `config/config.json` (runtime source of truth).
If the table ever disagrees with that file, trust the file.

### Body (body_node, ttyACM0–3)
| config_index | Motor name | VESC ID | Notes |
|---|---|---|---|
| 0 | Flipper Trasero | 2 | inverted=true |
| 1 | Track Izquierdo | 3 | duty_cycle mode |
| 2 | Track Derecho | 5 | inverted=true, duty_cycle mode |
| 3 | Flipper Delantero | 4 | |

Tank drive: `left = Y + X`, `right = Y - X`
Front flipper ← right_y, Rear flipper ← right_x

### Arm (arm_node, ttyACM4)
| config_index | Motor name | VESC ID | mode | rpm_limit | duty_cycle_limit |
|---|---|---|---|---|---|
| 4 | Cadera (Hip) | 0 | RPM | 3000 | 0.1 |
| 5 | Hombro (Shoulder) | 10 | duty_cycle | 3000 | 0.1 |
| 6 | Codo (Elbow) | 6 | duty_cycle | 3000 | 0.1 |
| 7 | Roll | 7 | RPM | 2000 | 0.1 |
| 8 | Pitch | 8 | duty_cycle | 1000 | 0.1 |
| 9 | Grip | 9 | duty_cycle | 1000 | 0.1 |

---

## VESC Serial Protocol

Library: `LibSerial` (libserial-dev)
Baudrate: 115200, timeout: 1000 ms
Frame: `[0x02][length][payload][CRC_H][CRC_L][0x03]`
CRC-16/CCITT (poly=0x1021, init=0)

Key packet IDs:
- `0x04` — COMM_GET_VALUES (request telemetry; response ≥54 bytes)
- `0x05` — COMM_SET_DUTY (int32 = duty × 100000, big-endian)
- `0x08` — COMM_SET_RPM (int32, big-endian)

Telemetry response offsets (COMM_GET_VALUES payload, 0-based after cmd byte):
- `[1-2]` temp_fet (÷10 → °C), `[5-8]` current_motor (÷100 → A),
  `[9-12]` current_in (÷100 → A), `[21-22]` duty (÷1000),
  `[23-26]` RPM (i32), `[27-28]` input_voltage (÷10 → V),
  `[53]` fault_code, `[54-57]` position (÷1000000 → deg), `[58]` motor_controller_id
- Minimum payload for full data: ≥59 bytes (check `payload[0] == 0x04`)

---

## Config File (runtime motor config)

Loaded by body_node and arm_node from:
`ament_index_cpp::get_package_share_directory("robot_pkg") + "/config/config.json"`

Updated at runtime via `/robot_config/update` topic. `global_settings` keys:
`control_mode`, `flipper_max_rpm`, `mobility_max_rpm`, `selected_flipper`.

---

## Multithreading Pattern (robot_pkg)

Each motor drive loop runs in its own callback group → parallel execution in
`MultiThreadedExecutor`. Joystick values shared between threads via `std::atomic<float>`.

```cpp
// Pattern used throughout body.cpp and arm.cpp
std::atomic<float> left_y_{0.0f};
rclcpp::CallbackGroup::SharedPtr cg_left_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
```

Telemetry timer: 50 ms period (20 Hz)
Drive timers: 10 ms period (100 Hz) per motor (arm timers currently 20 ms / 50 Hz)

---

## Docker

Image base: `osrf/ros:humble-desktop-full`, platform `linux/arm64`
(works on both Apple Silicon Mac and Jetson Orin — same arch)

```bash
docker compose build                  # build image
docker compose up <service>           # run a node

# On actual robot hardware (Linux/Jetson)
docker compose -f docker-compose.yml -f docker-compose.linux.yml up <service>

# After changing C++ source (faster than full rebuild)
docker compose -f docker-compose.yml -f docker-compose.dev.yml run --rm dev
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

## WiFi / DDS

Default DDS multicast fails on WiFi. Use unicast profile:

```bash
source fastdds/setup.sh   # sets FASTRTPS_DEFAULT_PROFILES_FILE
```

Edit `fastdds/fastdds_wifi.xml` to set ROBOT_IP and STATION_IP before sourcing.
Discovery port: 7412. Lease duration: 10 s. Initial announcements: 10 × 200 ms.

**Latency quick-wins** (see `LATENCY_IMPROVEMENTS.md` for full details):

1. Disable WiFi power saving on both machines (eliminates 10–50 ms spikes):
   ```bash
   sudo iw dev wlan0 set power_save off
   ```
2. Switch to CycloneDDS: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
3. On Jetson, lock clocks: `sudo nvpmodel -m 0 && sudo jetson_clocks`

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

Custom-installed (via install scripts): Vosk 0.3.45 (mic_pkg), ONNX Runtime 1.17.1
(detections_pkg, Ubuntu only — optional CUDA provider).

---

## Camera Config

IPs: `192.168.0.200–206` (7 cameras). Cameras 5–7 require auth (`admin:admin`).
Config: `src/camera_pkg/config/settings.json`
Hazmat models: `src/detections_pkg/config/models/hazmat_best.onnx` (YOLOv8),
`hazmat_resnet18.onnx` (ResNet-18 classifier)


## Approach

Think before acting. Read existing files before writing code.
Be concise in output but thorough in reasoning.
Prefer editing over rewriting whole files.
Do not re-read files you have already read unless the file may have changed.
Test your code before declaring done.
No sycophantic openers or closing fluff.
Keep solutions simple and direct.
User instructions always override this file.