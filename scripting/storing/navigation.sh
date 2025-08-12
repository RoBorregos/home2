SESSION_NAME="navigation-receptionist"

tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'cd /home/orin/dev/nav' C-m
tmux send-keys -t $SESSION_NAME 'bash run.sh navigation' C-m
tmux send-keys -t $SESSION_NAME 'ros2 launch nav_main navigation_composition.launch.py' C-m
