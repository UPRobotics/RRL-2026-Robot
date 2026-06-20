
WORKSPACE_DIR=~/Desktop/Programming_stuff/Cplusplus/RRL-2026-Robot

tmux new-session -d -s robot_core -c "$WORKSPACE_DIR"
tmux send-keys -t robot_core "source install/setup.bash && ros2 run robot_pkg body_node" Enter

tmux split-window -h -t robot_core -c "$WORKSPACE_DIR"
tmux send-keys -t robot_core "source install/setup.bash && ros2 run robot_pkg arm_node" Enter

tmux split-window -v -t robot_core -c "$WORKSPACE_DIR"
tmux send-keys -t robot_core "source install/setup.bash && ros2 run telemetry_pkg telemetry_node" Enter

tmux split-window -v -t robot_core -c "$WORKSPACE_DIR"
tmux send-keys -t robot_core "python3 run_camera_server.py" Enter

tmux select-layout -t robot_core tiled

tmux attach-session -t robot_core