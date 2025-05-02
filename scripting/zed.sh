SESSION_NAME="zed-wrapper"

tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 publish_tf:=false' C-m
