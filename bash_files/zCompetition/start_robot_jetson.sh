
WORKSPACE_DIR=~/RRL-KOREA-2026/RRL-2026-Robot

tmux new-session -d -s robot_jetson -c "$WORKSPACE_DIR"
tmux send-keys -t robot_jetson "source install/setup.bash && ros2 run robot_pkg body_node" Enter


tmux split-window -v -t robot_jetson -c "$WORKSPACE_DIR"
tmux send-keys -t robot_jetson "source install/setup.bash && ros2 run telemetry_pkg telemetry_node" Enter


tmux select-layout -t robot_jetson tiled

tmux attach-session -t robot_jetson