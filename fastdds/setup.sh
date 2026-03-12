#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# fastdds/setup.sh  —  Configura FastDDS para comunicación WiFi de baja latencia
# ──────────────────────────────────────────────────────────────────────────────
# Uso:
#   source fastdds/setup.sh           (desde la raíz del workspace)
# O agregar al final de ~/.bashrc (una vez que las IPs estén configuradas):
#   source /ruta/al/workspace/fastdds/setup.sh
#
# ANTES DE USAR:
#   Edita fastdds/fastdds_wifi.xml y reemplaza ROBOT_IP y STATION_IP
#   con las IPs reales de tu red WiFi.
# ──────────────────────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILE_FILE="$SCRIPT_DIR/fastdds_wifi.xml"

if [[ ! -f "$PROFILE_FILE" ]]; then
    echo "[FastDDS setup] ERROR: No se encontró $PROFILE_FILE"
    return 1
fi

# Validar que las IPs ya fueron configuradas
if grep -q "ROBOT_IP\|STATION_IP" "$PROFILE_FILE"; then
    echo ""
    echo "╔══════════════════════════════════════════════════════════╗"
    echo "║  ADVERTENCIA: fastdds/fastdds_wifi.xml contiene         ║"
    echo "║  IPs de marcador de posición (ROBOT_IP / STATION_IP).   ║"
    echo "║  Edita el archivo con las IPs reales antes de continuar.║"
    echo "╚══════════════════════════════════════════════════════════╝"
    echo ""
    # No retornar error — permitir continuar si el usuario lo sabe
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE_FILE"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "[FastDDS setup] Perfil WiFi activo: $PROFILE_FILE"
echo "[FastDDS setup] RMW: $RMW_IMPLEMENTATION"
