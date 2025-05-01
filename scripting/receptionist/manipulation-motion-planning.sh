SESSION_NAME="manipulation-receptionist-arm"

tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
tmux send-keys -t $SESSION_NAME 'bash run.sh manipulation' C-m
tmux send-keys -t $SESSION_NAME 'ros2 run frida_motion_planning motion_planning_server.py' C-m
