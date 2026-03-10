#!/bin/bash
###############################################################################
# install_ubuntu.sh
#
# Installs all dependencies for the ROS2 Humble workspace on
# Ubuntu 22.04 (x86_64) with ROS2 Humble pre-installed.
#
# Usage:  chmod +x install_ubuntu.sh && ./install_ubuntu.sh
###############################################################################
set -euo pipefail

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $*"; }
err()  { echo -e "${RED}[ERROR]${NC} $*"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}"

###############################################################################
# 1. System update
###############################################################################
log "Updating system packages..."
sudo apt-get update && sudo apt-get upgrade -y

###############################################################################
# 2. Core build tools & colcon
###############################################################################
log "Installing core build tools..."
sudo apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    wget \
    curl \
    unzip \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

###############################################################################
# 3. ROS2 Humble core packages (should already be installed)
###############################################################################
log "Ensuring ROS2 Humble core packages are present..."
sudo apt-get install -y \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-ros2launch \
    ros-humble-ament-cmake \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    ros-humble-joy

###############################################################################
# 4. camera_pkg dependencies
#    SDL2, SDL2_ttf, FFmpeg, spdlog, nlohmann-json
###############################################################################
log "Installing camera_pkg dependencies (SDL2, FFmpeg, spdlog, nlohmann-json)..."
sudo apt-get install -y \
    libsdl2-dev \
    libsdl2-2.0-0 \
    libsdl2-ttf-dev \
    libsdl2-ttf-2.0-0 \
    libavcodec-dev \
    libavcodec58 \
    libavformat-dev \
    libavformat58 \
    libswscale-dev \
    libswscale5 \
    libavutil-dev \
    libavutil56 \
    libspdlog-dev \
    nlohmann-json3-dev

###############################################################################
# 5. mic_pkg dependencies
#    ALSA, SDL2 (already above), Vosk speech recognition
###############################################################################
log "Installing mic_pkg dependencies (ALSA)..."
sudo apt-get install -y \
    libasound2-dev \
    libasound2

# --- Vosk C library (built from pre-compiled release) ---
VOSK_VERSION="0.3.45"
VOSK_INSTALLED=false

if [ -f /usr/local/lib/libvosk.so ] && [ -f /usr/local/include/vosk_api.h ]; then
    log "Vosk library already installed, skipping."
    VOSK_INSTALLED=true
fi

if [ "${VOSK_INSTALLED}" = false ]; then
    log "Installing Vosk C library v${VOSK_VERSION} for x86_64..."
    VOSK_ARCHIVE="vosk-linux-x86_64-${VOSK_VERSION}.zip"
    VOSK_URL="https://github.com/alphacep/vosk-api/releases/download/v${VOSK_VERSION}/${VOSK_ARCHIVE}"

    TMPDIR="$(mktemp -d)"
    cd "${TMPDIR}"
    wget -q "${VOSK_URL}" -O "${VOSK_ARCHIVE}" || {
        err "Failed to download Vosk. Please download manually from:"
        err "  ${VOSK_URL}"
        err "Then copy libvosk.so to /usr/local/lib/ and vosk_api.h to /usr/local/include/"
        cd "${WS_DIR}"
        rm -rf "${TMPDIR}"
        warn "Continuing without Vosk..."
    }

    if [ -f "${VOSK_ARCHIVE}" ]; then
        unzip -q "${VOSK_ARCHIVE}"
        VOSK_DIR="vosk-linux-x86_64-${VOSK_VERSION}"
        sudo cp "${VOSK_DIR}"/libvosk.so /usr/local/lib/
        sudo cp "${VOSK_DIR}"/vosk_api.h /usr/local/include/
        sudo ldconfig
        log "Vosk library installed successfully."
    fi

    cd "${WS_DIR}"
    rm -rf "${TMPDIR}"
fi

###############################################################################
# 6. Vosk speech models
###############################################################################
MODELS_DIR="${WS_DIR}/models"
mkdir -p "${MODELS_DIR}"

if [ ! -d "${MODELS_DIR}/vosk-model-small-en-us-0.15" ]; then
    log "Downloading Vosk English model..."
    cd "${MODELS_DIR}"
    wget -q "https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip" \
        -O vosk-model-small-en-us-0.15.zip || warn "Failed to download English model"
    if [ -f vosk-model-small-en-us-0.15.zip ]; then
        unzip -q vosk-model-small-en-us-0.15.zip
        rm vosk-model-small-en-us-0.15.zip
    fi
    cd "${WS_DIR}"
else
    log "Vosk English model already present."
fi

if [ ! -d "${MODELS_DIR}/vosk-model-small-es-0.42" ]; then
    log "Downloading Vosk Spanish model..."
    cd "${MODELS_DIR}"
    wget -q "https://alphacephei.com/vosk/models/vosk-model-small-es-0.42.zip" \
        -O vosk-model-small-es-0.42.zip || warn "Failed to download Spanish model"
    if [ -f vosk-model-small-es-0.42.zip ]; then
        unzip -q vosk-model-small-es-0.42.zip
        rm vosk-model-small-es-0.42.zip
    fi
    cd "${WS_DIR}"
else
    log "Vosk Spanish model already present."
fi

###############################################################################
# 7. thermal_pkg dependencies (OpenCV)
###############################################################################
log "Installing thermal_pkg dependencies (OpenCV)..."
sudo apt-get install -y libopencv-dev

###############################################################################
# 8. robot_pkg dependencies (libserial for VESC communication)
###############################################################################
log "Installing robot_pkg dependencies (libserial)..."
sudo apt-get install -y libserial-dev

###############################################################################
# 9. Python dependencies (for VESCRPMDutyCycle.py and test scripts)
###############################################################################
log "Installing Python dependencies..."
pip3 install --user pyserial

###############################################################################
# 10. Serial port permissions
###############################################################################
log "Adding user '${USER}' to 'dialout' group for serial port access..."
sudo usermod -aG dialout "${USER}" || warn "Could not add user to dialout group."

###############################################################################
# 11. Initialize / update rosdep
###############################################################################
log "Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init 2>/dev/null || warn "rosdep already initialized."
fi
rosdep update --rosdistro=humble

###############################################################################
# 12. Install remaining dependencies via rosdep
###############################################################################
log "Running rosdep install for workspace..."
cd "${WS_DIR}"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y || warn "Some rosdep keys could not be resolved."

###############################################################################
# 13. Build the workspace
###############################################################################
log "Building the workspace with colcon..."
cd "${WS_DIR}"
source /opt/ros/humble/setup.bash
colcon build --symlink-install 2>&1 | tee /tmp/colcon_build_output.log

BUILD_RC=${PIPESTATUS[0]}
if [ "${BUILD_RC}" -eq 0 ]; then
    log "Build completed successfully!"
else
    err "Build failed. Check /tmp/colcon_build_output.log for details."
fi

###############################################################################
# 14. Summary
###############################################################################
echo ""
log "============================================"
log " Installation complete (Ubuntu 22.04 x86_64)"
log "============================================"
echo ""
echo "  To use the workspace, run:"
echo "    source ${WS_DIR}/install/setup.bash"
echo ""
echo "  Available nodes:"
echo "    ros2 run camera_pkg camera_pkg_node"
echo "    ros2 run mic_pkg mic_transmitter_node"
echo "    ros2 run mic_pkg mic_receiver_node"
echo "    ros2 run robot_pkg arm_node"
echo "    ros2 run robot_pkg body_node"
echo "    ros2 run robot_pkg joystick_node"
echo "    ros2 run thermal_pkg thermal_camera_node"
echo "    ros2 run thermal_pkg thermal_display_node"
echo ""
echo "  NOTE: Log out and back in for 'dialout' group changes to take effect."
echo ""
