# SESSION_NAME="mr-follow-face"

# tmux new-session -d -s $SESSION_NAME
# tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
# tmux send-keys -t $SESSION_NAME 'bash run.sh manipulation' C-m
# tmux send-keys -t $SESSION_NAME 'ros2 run task_manager follow_face_node.py' C-m

# SESSION_NAME="mr-arm"

# tmux new-session -d -s $SESSION_NAME
# tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
# tmux send-keys -t $SESSION_NAME 'bash run.sh manipulation' C-m
# tmux send-keys -t $SESSION_NAME 'ros2 run frida_motion_planning motion_planning_server.py' C-m

# SESSION_NAME="mr-planning"

# tmux new-session -d -s $SESSION_NAME
# tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
# tmux send-keys -t $SESSION_NAME 'bash run.sh manipulation' C-m
# tmux send-keys -t $SESSION_NAME 'ros2 launch arm_pkg frida_moveit_config.launch.py robot_ip:=192.168.31.180' C-m

SESSION_NAME="manipulation-receptionist"

tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
tmux send-keys -t $SESSION_NAME 'bash run.sh manipulation' C-m
tmux send-keys -t $SESSION_NAME 'ros2 launch manipulation_general gpsr.launch.py' C-m
