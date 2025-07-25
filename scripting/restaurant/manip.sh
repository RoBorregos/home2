SESSION_NAME="manip"

tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
tmux send-keys -t $SESSION_NAME 'bash run.sh manipulation' C-m
tmux send-keys -t $SESSION_NAME 'ros2 launch manipulation_general carry.launch.py' C-m
