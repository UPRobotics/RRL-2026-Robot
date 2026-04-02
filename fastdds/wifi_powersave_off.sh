#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# fastdds/wifi_powersave_off.sh — Disable WiFi power saving for low latency
# ──────────────────────────────────────────────────────────────────────────────
# Run on BOTH the robot (Jetson) and the ground station before a match.
# WiFi power saving can add 10–50 ms latency spikes when the adapter sleeps
# between packets.
#
# Usage:
#   sudo bash fastdds/wifi_powersave_off.sh
#
# To make permanent (NetworkManager):
#   sudo cp fastdds/wifi_powersave_off.conf /etc/NetworkManager/conf.d/
#   sudo systemctl restart NetworkManager
# ──────────────────────────────────────────────────────────────────────────────

set -e

# Detect active WiFi interface automatically
IFACE=$(iw dev 2>/dev/null | awk '/Interface/{print $2}' | head -1)

if [[ -z "$IFACE" ]]; then
    echo "ERROR: No WiFi interface found."
    exit 1
fi

echo "WiFi interface: $IFACE"

# Disable power saving
iwconfig "$IFACE" power off 2>/dev/null || true
iw dev "$IFACE" set power_save off

# Verify
STATUS=$(iw dev "$IFACE" get power_save 2>/dev/null || iwconfig "$IFACE" | grep -o "Power Management:[a-z]*")
echo "Power save status: $STATUS"

# Jetson-specific: lock clocks to max performance
if command -v nvpmodel &>/dev/null; then
    echo "Jetson detected — setting max performance mode..."
    nvpmodel -m 0
    jetson_clocks
    echo "Jetson clocks locked."
fi

echo "Done. Run this script after every reboot, or install the NetworkManager conf for persistence."
