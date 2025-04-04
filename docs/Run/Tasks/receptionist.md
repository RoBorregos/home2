# Receptionist

## Vision
Run container. From the root directory (home2), run:
```bash
./run.sh vision --receptionist
```

## Manipulation
Run container. From the root directory (home2), run:
```bash
./run.sh manipulation
```

## HRI
Run containers. From `docker/hri` run:
```bash
docker compose up
```

Note: ollama models are downloaded and mounted on the repo. Run the repo at the path `/home/orin/home2` to avoid having to re-download the models.

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