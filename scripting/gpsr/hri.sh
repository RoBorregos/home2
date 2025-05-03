SESSION_NAME="hri-gpsr"

tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
tmux send-keys -t $SESSION_NAME 'bash run.sh hri --gpsr' C-m
tmux send-keys -t $SESSION_NAME "$SUDO_PASSWORD" C-m
