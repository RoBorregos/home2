
SESSION_NAME="nav"

tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
tmux send-keys -t $SESSION_NAME 'bash run.sh navigation' C-m
tmux send-keys -t $SESSION_NAME 'ros2 launch nav_main carry_my.launch.py' 