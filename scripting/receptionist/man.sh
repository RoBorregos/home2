SESSION_NAME="manipulation-receptionist-planning"

tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
tmux send-keys -t $SESSION_NAME 'bash run.sh manipulation' C-m
tmux send-keys -t $SESSION_NAME 'groups' C-m

# tmux send-keys -t $SESSION_NAME 'ros2 launch arm_pkg frida_moveit_config.launch.py robot_ip:=192.168.31.180' C-m
