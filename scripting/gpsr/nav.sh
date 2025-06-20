SESSION_NAME="nav-gpsr"
SESSION_NAME_2="nav-composable"

tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
tmux send-keys -t $SESSION_NAME 'bash run.sh navigation' C-m
tmux send-keys -t $SESSION_NAME 'ros2 launch nav_main receptionist.launch.py' 

tmux new-session -d -s $SESSION_NAME_2
tmux send-keys -t $SESSION_NAME_2 'cd ../../' C-m
tmux send-keys -t $SESSION_NAME_2 'bash run.sh navigation' C-m 
tmux send-keys -t $SESSION_NAME_2 'ros2 launch nav_main composabletest.launch.py' 