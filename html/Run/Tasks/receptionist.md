# Receptionist

## ZED
Run container. From the root directory (home2), run:
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed publish_tf:=false
```

## Integration
Run container. From the root directory (home2), run:
```bash
./run.sh integration --receptionist
``` 

## Vision
Run container. From the root directory (home2), run:
```bash
./run.sh vision --receptionist
```

## HRI
Run containers. From the root directory (home2), run:
```bash
./run.sh hri --receptionist
```

## Navigation
!! WARNING !!

"Frida_moveit_config" should be running from the manipulation namespace in order for navigation to work.

ZED should be running too.

Run container. From the root directory (home2), run:
```bash
./run.sh navigation --receptionist
```
- Check the RViz window that pops up on the Orin. After a few seconds, you should see the TF and the local costmap moving. If not, restart the Docker container and verify that both the ZED and Frida MoveIt config are running.
- Check the position of the TF in the map. If it’s not correctly positioned, use the "2D Estimate Pose" tool located at the top of RViz. Click on the map where the robot should be, and remember to hold the click and drag to set the orientation angle as well.

## Manipulation
Run container. From the root directory (home2), run:
```bash
./run.sh manipulation
```

Note: ollama models are downloaded and mounted on the repo. Run the repo at the path `/home/orin/home2` to avoid having to re-download the models.
### Launch 
To launch the manipulation stack, run the following command:
```bash
ros2 launch manipulation_general receptionist.launch.py
```

The launch runs the following nodes:
### Arm bringup
To test on the real robot, run the launch to bring up the robot and MoveIt planning:
```bash
ros2 launch arm_pkg frida_moveit_config.launch.py robot_ip:=192.168.31.180
```

### Motion Planning
Check that the motion planning interface is running:
```bash
ros2 run frida_motion_planning motion_planning_server.py
```

### Follow face
To follow a face, run the following command:
```bash
ros2 run task_manager follow_face_node.py
```