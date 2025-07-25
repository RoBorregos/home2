
SESSION_NAME="object-detector"

tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'cd ../../' C-m
tmux send-keys -t $SESSION_NAME 'bash run.sh vision --storing-groceries' C-m
# tmux send-keys -t $SESSION_NAME 'ros2 launch object_detector_2d object_detector_node.launch.py yolo_model_path:=src/home2/vision/packages/object_detector_2d/models/yolo11classes.pt'
