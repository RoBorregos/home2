SESSION_NAME="manipulation-receptionist-follow-face"

tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
tmux send-keys -t $SESSION_NAME 'bash run.sh manipulation' C-m
tmux send-keys -t $SESSION_NAME 'ros2 run task_manager follow_face_node.py'
