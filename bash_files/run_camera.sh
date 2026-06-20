#!/usr/bin/env bash
set -euo pipefail

# Repo-relative launcher for camera_pkg node
# Usage:
#   ./run_camera.sh                # runs with default: --cpu --debug
#   ./run_camera.sh --gpu --debug  # custom flags passed to node
#   ./run_camera.sh --gdb          # run under gdb with default flags

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_SETUP="$REPO_ROOT/install/setup.bash"
NODE="$REPO_ROOT/install/camera_pkg/lib/camera_pkg/camera_pkg_node"
CONFIG_DIR="$REPO_ROOT/install/camera_pkg/share/camera_pkg/config"

if [ ! -f "$INSTALL_SETUP" ]; then
  echo "Error: install/setup.bash not found. Build the workspace first." >&2
  exit 1
fi

if [ ! -x "$NODE" ]; then
  echo "Warning: node binary not found or not executable: $NODE" >&2
fi

# Default arguments when no explicit args are provided
DEFAULT_ARGS=(--cpu --debug --ros-args -r __node:=camera_pkg -p "config_dir:=$CONFIG_DIR")

if [ "$#" -eq 0 ]; then
  ARGS=("${DEFAULT_ARGS[@]}")
else
  if [ "$1" = "--gdb" ]; then
    shift
    if [ "$#" -gt 0 ]; then
      ARGS=("$@")
    else
      ARGS=("${DEFAULT_ARGS[@]}")
    fi
    # Temporarily disable 'unset variable' errors while sourcing ROS/colcon setup
    set +u
    source "$INSTALL_SETUP"
    set -u
    exec gdb --args "$NODE" "${ARGS[@]}"
  else
    ARGS=("$@")
  fi
fi

# Temporarily disable 'unset variable' errors while sourcing ROS/colcon setup
set +u
source "$INSTALL_SETUP"
set -u
exec "$NODE" "${ARGS[@]}"
