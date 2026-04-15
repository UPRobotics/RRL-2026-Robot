# RoboticaWS

ROS 2 Humble workspace for a mobile rescue robot featuring VESC motor control, multi-camera streaming, hazmat/QR detection, speech recognition, thermal imaging, and magnetometer sensing.

## Supported Platforms

| Platform | Architecture | Packages |
|----------|-------------|----------|
| Ubuntu 22.04 LTS | x86_64 | All |
| NVIDIA Jetson Orin (JetPack) | aarch64 | All except `camera_pkg` and `detections_pkg` |

## Installation

```bash
git clone <repo-url> roboticaWS && cd roboticaWS

# Ubuntu desktop
chmod +x install_ubuntu.sh && ./install_ubuntu.sh

# Jetson Orin
chmod +x install_jetson_orin.sh && ./install_jetson_orin.sh
```

The install scripts handle all system dependencies, speech models, ONNX Runtime (Ubuntu), Vosk, workspace build, and serial port permissions. Log out and back in after install for `dialout` group changes to take effect.

To source the workspace in any new terminal:

```bash
source install/setup.bash
```

## Packages

### robot_pkg

Controls the robot chassis, flippers, and arm through VESC motor controllers over serial.

| Node | Description |
|------|-------------|
| `body_node` | Drives left/right motors and left/right flippers |
| `arm_node` | Controls the robotic arm motor |
| `joystick_node` | Reads gamepad input and publishes control commands |

```bash
ros2 run robot_pkg body_node
ros2 run robot_pkg arm_node
ros2 run robot_pkg joystick_node
```

**Hardware:** 4 VESC controllers for body/flippers (`/dev/ttyACM0`–`/dev/ttyACM3`) + 1 for arm. Joystick via `sensor_msgs/Joy`.

---

### camera_pkg *(desktop only)*

Multi-camera viewer supporting up to 7 RTSP IP cameras and a thermal camera feed from the `/thermal_data` ROS topic.

```bash
ros2 launch camera_pkg camera_pkg.launch.py
```

**Config:** `config/settings.json` — camera IPs, display layout, console settings.

---

### detections_pkg *(desktop only)*

Real-time detection pipeline on an RTSP camera stream with a visual display.

| Detector | Method |
|----------|--------|
| QR codes | ZBar library |
| Motion | Optical-flow frame differencing (OpenCV) |
| Hazmat signs | YOLOv8 detection + ResNet-18 classification (ONNX Runtime, optional CUDA) |

```bash
ros2 launch detections_pkg detections.launch.py
```

**Config:** `config/detections_config.json` — RTSP URL, detection intervals, confidence thresholds.
**Models:** `config/models/hazmat_best.onnx`, `config/models/hazmat_resnet18.onnx`.

---

### mic_pkg

USB microphone capture with offline speech-to-text using Vosk. Supports English and Spanish with live transcript display.

| Node | Description |
|------|-------------|
| `mic_transmitter_node` | Captures audio from USB mic (auto-detected by vendor/product ID) |
| `mic_receiver_node` | Transcribes audio to text via Vosk, displays live transcript |

```bash
ros2 launch mic_pkg mic_transmitter.launch.py
```

**Parameters:** `device`, `usb_vendor_id`, `usb_product_id`, `sample_rate`, `lang` (`en`/`es`).
**Models:** `models/vosk-model-small-en-us-0.15`, `models/vosk-model-small-es-0.42` (downloaded by install script).

---

### thermal_pkg

Reads a MLX90640 thermal sensor over serial and displays a real-time heat map with interactive temperature probing.

| Node | Description |
|------|-------------|
| `thermal_camera_node` | Reads sensor data, publishes `Float32MultiArray` on `/thermal_data` |
| `thermal_display_node` | Renders heat map via OpenCV; click to probe temperature, press `R` to clear |

```bash
ros2 launch thermal_pkg thermal_camera.launch.py serial_port:=/dev/ttyUSB0
```

---

### magnetometer_pkg

Python-based serial magnetometer reader with a Tkinter/Matplotlib real-time visualizer.

| Node | Description |
|------|-------------|
| `magnetometer_sender` | Reads 3-axis magnetometer via serial, publishes JSON on ROS topic |
| `magnetometer_receiver` | GUI showing live X/Y/Z/magnitude values and 60-second history graph |

```bash
ros2 launch magnetometer_pkg magnetometer.launch.py
```

**Parameters:** `port` (default `/dev/ttyACM0`), `baudrate` (default `115200`), `timeout`.

---

### control_pkg

Placeholder package reserved for future high-level control logic.

## Project Structure

```
roboticaWS/
├── src/
├   |── camera_pkg/           # Multi-camera RTSP viewer (desktop)
│   ├── control_pkg/          # Control logic (placeholder)
│   ├── detections_pkg/       # QR, motion & hazmat detection (desktop)
│   ├── fastlio_pkg/          # LiDAR-Inertial SLAM
│   ├── livox_ros_driver2/    # Official Livox LiDAR driver
│   ├── magnetometer_pkg/     # Magnetometer reader & visualizer
│   ├── mic_pkg/              # Microphone capture & speech recognition
│   ├── octomap_mapping/      # OctoMap mapping
│   ├── octomap_msgs/         # OctoMap messages
│   ├── robot_pkg/            # VESC motor control
│   └── thermal_pkg/          # MLX90640 thermal camera
├── models/                   # Vosk speech recognition models
├── detections/               # Saved detection outputs (QR, hazmat)
├── install_ubuntu.sh         # Full install script for Ubuntu 22.04
├── install_jetson_orin.sh    # Install script for Jetson Orin
├── build/
├── install/
└── log/
```

## Dependencies

### System Libraries

| Library | Apt Package | Used By |
|---------|-------------|---------|
| SDL2 | `libsdl2-dev` | camera_pkg, detections_pkg, mic_pkg |
| SDL2_ttf | `libsdl2-ttf-dev` | camera_pkg, detections_pkg, mic_pkg |
| FFmpeg | `libavcodec-dev` `libavformat-dev` `libswscale-dev` `libavutil-dev` | camera_pkg, detections_pkg |
| OpenCV | `libopencv-dev` | detections_pkg, thermal_pkg |
| ZBar | `libzbar-dev` | detections_pkg |
| spdlog | `libspdlog-dev` | camera_pkg, detections_pkg |
| nlohmann-json | `nlohmann-json3-dev` | camera_pkg, detections_pkg |
| ALSA | `libasound2-dev` | mic_pkg |
| LibSerial | `libserial-dev` | robot_pkg |
| Matplotlib | `python3-matplotlib` | magnetometer_pkg |
| Tkinter | `python3-tk` | magnetometer_pkg |

### Custom-Installed Libraries

| Library | Version | Used By | Notes |
|---------|---------|---------|-------|
| Vosk | 0.3.45 | mic_pkg | Installed from GitHub release (x86_64 or aarch64) |
| ONNX Runtime | 1.17.1 | detections_pkg | Ubuntu only; optional CUDA provider for GPU inference |

### ROS 2 Packages

`rclcpp`, `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `ament_index_cpp`, `joy`

### Python

`pyserial`

## Building

```bash
# Full build
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Single package
colcon build --packages-select robot_pkg

# Jetson (skip desktop-only packages)
colcon build --symlink-install --packages-ignore camera_pkg detections_pkg
```

## Serial Port Setup

The install scripts add your user to the `dialout` group. If you still get permission errors:

```bash
sudo usermod -aG dialout $USER   # then log out and back in
```

## License

MIT — see [LICENSE](LICENSE) for details.
