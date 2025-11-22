#!/bin/bash
source /opt/ros/humble/setup.bash

# Start ROS web video server silently
nohup ros2 run web_video_server web_video_server > /dev/null 2>&1 &

# Start your main app
npm run start