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
- `compile_computer.sh` — **full `rm -rf build install log`** then `colcon build` (all packages)
- `compile_jetson.sh` — same full clean, ignores `camera_pkg detections_pkg`
- `execute_computer.sh` — sources `install/setup.bash` then launches `control_pkg joystick.launch.py`
- `execute_robot.sh` — sources `install/setup.bash` then launches `robot_pkg robot.launch.py`

> Both compile scripts do a **destructive full clean rebuild** every time. Use `colcon build --packages-select <pkg>` for incremental iteration.

## Running Nodes

```bash
# Ground station (operator side)
ros2 launch control_pkg joystick.launch.py     # joy_node + joystick_node
ros2 run telemetry_ui_pkg telemetry_ui_node    # SDL2 dashboard

# Robot side
ros2 launch robot_pkg robot.launch.py          # arm_node + body_node + telemetry_node (from telemetry_pkg)

# Separate body/arm launches
ros2 launch robot_pkg body.launch.py
ros2 launch robot_pkg arm.launch.py

# Simulation (no hardware — motor_sim_node + telemetry_node + telemetry_ui)
ros2 launch robot_pkg robot_sim.launch.py [num_motors:=10]

# Individual nodes
ros2 run robot_pkg body_node
ros2 run robot_pkg arm_node
ros2 launch thermal_pkg thermal_camera.launch.py serial_port:=/dev/ttyUSB0
ros2 launch magnetometer_pkg magnetometer.launch.py

# FastDDS unicast (required over WiFi — run before launching anything)
source fastdds/setup.sh           # FastDDS; edit fastdds/fastdds_wifi.xml first
source fastdds/setup_cyclone.sh   # CycloneDDS alternative; edit fastdds/cyclonedds.xml first

# Pre-match: disable WiFi power save (also runs nvpmodel+jetson_clocks on Jetson)
sudo bash fastdds/wifi_powersave_off.sh
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

Mode toggle: Xbox **B button** (buttons[1]) / PS4 **Circle** — rising-edge, toggles between 0=MOVEMENT and 1=ARM.

**Controller auto-detection**: `controller_type` parameter defaults to `"auto"`. On the first Joy message, if `axes[4] > 0.5` (PS4 triggers start at 1.0; Xbox at 0.0) → PS4 layout; otherwise Xbox. Override with `controller_type:="ps4"` or `"xbox"`. `left_x` is **negated** before publishing.

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

Trigger normalization: raw joy value 1.0 (unpressed) → 0.0, -1.0 (full press) → 1.0.

### Motor telemetry (robot_msgs/MotorTelemetry, BEST_EFFORT KeepLast(5))

| Topic | config_index |
|-------|-------------|
| `/body_left_flipper/telemetry` | 0 |
| `/body_left/telemetry` | 1 |
| `/body_right/telemetry` | 2 |
| `/body_right_flipper/telemetry` | 3 |
| `/arm_hip/telemetry` | 4 |
| `/arm_shoulder/telemetry` | 5 |
| `/arm_elbow/telemetry` | 6 |
| `/arm_roll/telemetry` | 7 |
| `/arm_pitch/telemetry` | 8 |
| `/arm_claw/telemetry` | 9 |

### Config update pipeline (RELIABLE KeepLast(10))
```
/ground_station/motor_config  (robot_msgs/MotorConfig)
  └─ telemetry_node (telemetry_pkg)
       └─ /robot_config/update  (robot_msgs/MotorConfig)
            └─ body_node / arm_node (robot_pkg)

/ground_station/dpad_config   (robot_msgs/DPadConfig, RELIABLE KeepLast(10))
  └─ body_node / arm_node  — updates rpm/duty limits live
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
| config_index | Motor name | ttyACM | Notes |
|---|---|---|---|
| 0 | Flipper Trasero (left_flipper) | ACM* | inverted=true |
| 1 | Track Izquierdo (left) | ACM* | |
| 2 | Track Derecho (right) | ACM* | inverted=true |
| 3 | Flipper Delantero (right_flipper) | ACM* | |

> **Note:** VESC IDs and limits are stored in `src/robot_pkg/config/config.json`, which is the runtime source of truth. The IDs in config.json may differ from the hardware VESC firmware IDs — always check the file.

Tank drive: `left = Y + X`, `right = Y - X`

Flipper mapping (computed in body_node, not control_pkg):
```
front_flipper (Delantero) ← right_y   (stick up/down)
rear_flipper  (Trasero)   ← right_x   (stick left/right)
diagonal right stick      → both move independently
```
In ARM mode, flipper atomics are zeroed before updating mode to avoid torn reads.

**Motor groups** (used in telemetry_ui for bulk config):
- `Flippers`: config_index {0, 3}
- `Tracks`: config_index {1, 2}

### Arm (arm_node, ttyACM4)
| config_index | Motor name | mode | rpm_limit | ARM axis |
|---|---|---|---|---|
| 4 | Hip | RPM | 3000 | left_x |
| 5 | Shoulder | duty_cycle | 3000 | left_y |
| 6 | Elbow | duty_cycle | 3000 | right_y |
| 7 | Roll | RPM | 2000 | right_x |
| 8 | Pitch | duty_cycle | 1000 | trigger_right − trigger_left |
| 9 | Grip | duty_cycle | 1000 | bumper_right=+1, bumper_left=−1 |

All arm `duty_cycle_limit = 0.1` (from config.json). Deadzone: **0.1** — inputs below 10% are zeroed.

### Motor `enabled` flag

Each motor in `config.json` has an `enabled` field. When `false`, the node skips `autoConnect` and publishes a phantom telemetry entry so the UI slot still renders. Check `config.json` when a motor appears connected in the UI but doesn't respond — it may be disabled.

---

## VESC Serial Protocol

Library: `LibSerial` (libserial-dev)
Baudrate: 115200, timeout: 1000 ms
Frame: `[0x02][length][payload][CRC_H][CRC_L][0x03]`

Key packet IDs:
- `0x04` = COMM_GET_VALUES (request telemetry)
- `0x05` = COMM_SET_DUTY (duty cycle × 100000 as big-endian int32)
- `0x08` = COMM_SET_RPM (big-endian int32)

### VESC autoConnect

`VESC::autoConnect(hint)` scans `/dev/ttyACM*` and identifies each motor by probing its
firmware ID via `COMM_GET_VALUES`. Key details:
- A static `VESC::scan_mutex_` serializes all autoConnect calls across motor instances,
  preventing two motors from claiming the same port simultaneously.
- If `hint` (last known port path) is provided, it is tried first before the full scan —
  this speeds up reconnection on restart. The port is cached back to config.json after
  a successful connect.
- On disconnect, `running` is set false; `isConnected()` catches exceptions from a
  closed port and returns false safely.

Standalone VESC test tool (no ROS required): `src/robot_pkg/src/pytest/pytest.py`
Edit `SERIAL_PORTS` dict at the top to target specific ports.

---

## Config File (runtime motor config)

Loaded by body_node and arm_node from:
`ament_index_cpp::get_package_share_directory("robot_pkg") + "/config/config.json"`

Updated at runtime via `/robot_config/update` topic. Config writes are handled by a
dedicated background thread + `condition_variable` in each node (config_thread_) so
disk I/O never blocks the drive timers.

---

## Safety: Watchdog

Both `body_node` and `arm_node` have a 100 ms watchdog timer. If no `/control/input` arrives for **200 ms**, all joystick command atomics are zeroed and motors stop. This fires within 200 ms of a WiFi drop. The watchdog does not arm until the first message arrives.

## D-Pad Override

D-pad is a priority override that applies in **any** mode (MOVEMENT or ARM). When `dpad_x != 0` or `dpad_y != 0`, body drive ignores the joystick and uses D-pad values with `DPadLimits` (default rpm=500, duty=0.10). These limits are updated via `/ground_station/dpad_config` and immediately persisted to `config.json`. D-pad does not affect arm motors.

---

## Multithreading Pattern (robot_pkg)

Each motor drive loop runs in its own `MutuallyExclusive` callback group → parallel
execution in `MultiThreadedExecutor`. Joystick values shared between threads via
`std::atomic<float>`.

```cpp
std::atomic<float> left_y_{0.0f};
rclcpp::CallbackGroup::SharedPtr cg_left_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
```

Telemetry timer: 20 ms (50 Hz). Drive timers: 10 ms (100 Hz) per motor.

---

## Config File Coordination

`body_node` owns indices 0–3 and `arm_node` owns indices 4–9, but **both read and write the same `config.json`**. Before each write, the node re-reads the file to avoid clobbering the other node's section. There is no file-level lock; simultaneous saves from both nodes (triggered by concurrent config updates) have a narrow TOCTOU race.

`telemetry_pkg` has its **own separate** `telemetry_pkg/config/config.json` — it reads `arm_max_rpm` from there and does not receive runtime config updates.

---

## telemetry_ui_pkg

SDL2 dashboard (`src/telemetry_ui_pkg/src/main_window.cpp`). Uses `rclcpp::spin_some()`
in a manual event loop (not MultiThreadedExecutor) so it can interleave ROS callbacks
with SDL rendering.

Font hardcoded to `/usr/share/fonts/truetype/freefont/FreeMono.ttf`. Install with:
```bash
sudo apt install fonts-freefont-ttf
```

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

Permanent tuning: `sudo cp fastdds/sysctl_udp.conf /etc/sysctl.d/99-ros2-udp.conf` (UDP buffer sizes) and `sudo cp fastdds/wifi_powersave_off.conf /etc/NetworkManager/conf.d/` (power save).

---

## Camera Config

IPs: `192.168.0.200–206` (7 cameras). Cameras 5–7 require auth (`admin:admin`).
Config: `src/camera_pkg/config/settings.json`
Hazmat models: `src/detections_pkg/config/models/hazmat_best.onnx` (YOLOv8),
`hazmat_resnet18.onnx` (ResNet-18 classifier)

---

## Additional Packages (LiDAR / Mapping)

The following packages exist in `src/` but are not part of the standard robot/ground-station launch:

| Package | What it does |
|---------|-------------|
| `fastlio_pkg` | LiDAR-Inertial SLAM (FAST-LIO) |
| `livox_ros_driver2` | Livox LiDAR driver |
| `octomap_mapping` | OctoMap 3D mapping |
| `octomap_msgs` | OctoMap messages |

`colcon build` without `--packages-select` will build these. `compile_computer.sh` / `compile_jetson.sh` pass `--allow-overriding octomap_msgs` to handle the upstream conflict.

---

## Key System Dependencies

| Package | apt name |
|---------|---------|
| Serial comm | libserial-dev |
| JSON | nlohmann-json3-dev |
| Joystick driver | ros-humble-joy |
| SDL2 graphics | libsdl2-dev, libsdl2-ttf-dev |
| SDL2 font | fonts-freefont-ttf |
| FFmpeg (RTSP) | libavcodec-dev, libavformat-dev, libswscale-dev, libavutil-dev |
| Logging | libspdlog-dev |
| Computer vision | libopencv-dev |
| QR detection | libzbar-dev |
| Audio | libasound2-dev |
| Python serial | python3-serial |
| Python GUI | python3-matplotlib, python3-tk |
