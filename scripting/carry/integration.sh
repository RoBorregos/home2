#help_me_carry.py

SESSION_NAME="integration"

tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
tmux send-keys -t $SESSION_NAME 'bash run.sh integration' C-m
tmux send-keys -t $SESSION_NAME 'ros2 run task_manager help_me_carry.py' C-m
