#!/bin/bash

# Define the workspace directory to avoid repeating it
WORKSPACE_DIR=~/Desktop/Programming_stuff/Cplusplus/RRL-2026-Robot

# Start a new tmux session named "robot_core" and start the body_node in the first pane
tmux new-session -d -s robot_core -c "$WORKSPACE_DIR"
tmux send-keys -t robot_core "source install/setup.bash && ros2 run robot_pkg body_node" Enter

# Split the window horizontally to create a second pane for the arm_node
tmux split-window -h -t robot_core -c "$WORKSPACE_DIR"
tmux send-keys -t robot_core "source install/setup.bash && ros2 run robot_pkg arm_node" Enter

# Split the second pane vertically to create a third pane for the telemetry_node
tmux split-window -v -t robot_core -c "$WORKSPACE_DIR"
tmux send-keys -t robot_core "source install/setup.bash && ros2 run telemetry_pkg telemetry_node" Enter

# Set the layout to look nice and even (tiled)
tmux select-layout -t robot_core tiled

# Attach to the session so you can see all 3 terminals running
tmux attach-session -t robot_core