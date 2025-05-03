SESSION_NAME="integration-receptionist"

tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
tmux send-keys -t $SESSION_NAME 'bash run.sh integration' C-m
tmux send-keys -t $SESSION_NAME 'ros2 run task_manager gpsr_task_manager.py'
