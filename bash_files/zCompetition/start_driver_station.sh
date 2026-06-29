
WORKSPACE_DIR=~/RRL-KOREA-2026/RRL-2026-Robot

tmux new-session -d -s robot_core -c "$WORKSPACE_DIR"
tmux send-keys -t robot_core "source install/setup.bash && ros2 run telemetry_ui_pkg telemetry_ui_node" Enter

tmux split-window -h -t robot_core -c "$WORKSPACE_DIR"
tmux send-keys -t robot_core "source install/setup.bash && ros2 run control_pkg joystick_node" Enter

tmux split-window -v -t robot_core -c "$WORKSPACE_DIR"
tmux send-keys -t robot_core "source install/setup.bash && ros2 run joy joy_node" Enter

tmux select-layout -t robot_core tiled

tmux attach-session -t robot_core