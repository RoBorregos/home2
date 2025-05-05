SESSION_NAME="navigation-receptionist"

tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
tmux send-keys -t $SESSION_NAME 'bash run.sh navigation --receptionist' 