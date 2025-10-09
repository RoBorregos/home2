SESSION_NAME="hri"

tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
tmux send-keys -t $SESSION_NAME 'bash run.sh hri --carry' C-m
tmux send-keys -t $SESSION_NAME "$SUDO_PASSWORD" C-m
