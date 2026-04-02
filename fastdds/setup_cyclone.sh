#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# fastdds/setup_cyclone.sh  —  CycloneDDS low-latency WiFi profile
# ──────────────────────────────────────────────────────────────────────────────
# Usage:
#   source fastdds/setup_cyclone.sh       (from workspace root)
#
# BEFORE USE:
#   Edit fastdds/cyclonedds.xml and replace ROBOT_IP and STATION_IP
#   with the real IPs on your WiFi network.
#
# Install CycloneDDS RMW if not already installed:
#   sudo apt install ros-humble-rmw-cyclonedds-cpp
# ──────────────────────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CYCLONE_FILE="$SCRIPT_DIR/cyclonedds.xml"

if [[ ! -f "$CYCLONE_FILE" ]]; then
    echo "[CycloneDDS setup] ERROR: $CYCLONE_FILE not found"
    return 1
fi

if grep -q "ROBOT_IP\|STATION_IP" "$CYCLONE_FILE"; then
    echo ""
    echo "╔══════════════════════════════════════════════════════════╗"
    echo "║  WARNING: fastdds/cyclonedds.xml still contains         ║"
    echo "║  placeholder IPs (ROBOT_IP / STATION_IP).               ║"
    echo "║  Edit the file with real IPs before continuing.         ║"
    echo "╚══════════════════════════════════════════════════════════╝"
    echo ""
fi

# Check that the CycloneDDS RMW is installed
if ! ros2 pkg list 2>/dev/null | grep -q "rmw_cyclonedds_cpp"; then
    echo "[CycloneDDS setup] WARNING: rmw_cyclonedds_cpp not found."
    echo "  Install with: sudo apt install ros-humble-rmw-cyclonedds-cpp"
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONE_FILE}"

echo "[CycloneDDS setup] RMW : $RMW_IMPLEMENTATION"
echo "[CycloneDDS setup] URI : $CYCLONEDDS_URI"
