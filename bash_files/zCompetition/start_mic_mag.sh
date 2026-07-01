
WORKSPACE_DIR=~/RRL-KOREA-2026/RRL-2026-Robot

tmux new-session -d -s ui_session -c "$WORKSPACE_DIR"
tmux send-keys -t ui_session "source install/setup.bash && ros2 run mic_pkg mic_receiver_node" Enter

tmux split-window -h -t ui_session -c "$WORKSPACE_DIR"
tmux send-keys -t ui_session "source install/setup.bash && ros2 run magnetometer_pkg magnetometer_receiver" Enter

tmux select-layout -t ui_session tiled

tmux attach-session -t ui_session
