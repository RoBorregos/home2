SESSION_NAME="integration-storing"

tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
tmux send-keys -t $SESSION_NAME 'bash run.sh integration' C-m
tmux send-keys -t $SESSION_NAME 'ros2 run task_manager point_transformer.py' C-m 