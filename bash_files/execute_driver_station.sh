#!/bin/bash

# Define the workspace directory to avoid repeating it
WORKSPACE_DIR=~/Desktop/Programming_stuff/Cplusplus/RRL-2026-Robot

SESSION_NAME="driver_station"

tmux kill-session -t "$SESSION_NAME" 2>/dev/null

tmux new-session -d -s "$SESSION_NAME" -c "$WORKSPACE_DIR"
tmux send-keys -t "$SESSION_NAME" "source install/setup.bash && ros2 run control_pkg joystick_node" Enter

tmux split-window -h -t "$SESSION_NAME" -c "$WORKSPACE_DIR"
tmux send-keys -t "$SESSION_NAME" "source install/setup.bash && ros2 run telemetry_ui_pkg telemetry_ui_node" Enter

tmux split-window -v -t "$SESSION_NAME" -c "$WORKSPACE_DIR"
tmux send-keys -t "$SESSION_NAME" "source install/setup.bash && ros2 run joy joy_node" Enter

tmux select-layout -t "$SESSION_NAME" tiled

tmux attach-session -t "$SESSION_NAME"