# Manipulation
To run or test the modules it is necessary to have Ubuntu 22.04. Alternatively, you can use the docker setup provided in this repository.

## Docker setup
To make a container and open a shell for developing or testing the vision package, use the script `docker/manipulation/run.sh`. This can be accessed from the general script `./run.sh` in the root directory by passing the argument vision. This will first build the base image according to your system (cpu, cuda or jetson) as well as the image for the vision module and then run the container.

In root directory (home2), run:
```bash
./run.sh manipulation
```

If the script is not executable, run:
```bash
chmod +x run.sh
```

If the camera is not available run the following command before running the `/run.sh vision` command: 
```bash
sudo chmod 666 /dev/video0
```

# Running the vision module
Once in the docker workspace or using ROS2 in the home2 directory, run the following commands:

### Build
The manipulation packages require building several packages stored as submodules. Remember to pull them from their repositories:

```bash
git submodule update --init --recursive
```

### Real Robot and Simulation bringup
To test on simulation, run the launch to bring up the simulation environment and MoveIt planning:
```bash
ros2 launch frida_description moveit.launch.py add_gripper:=true add_realsense_d435i:=true
```

To test on the real robot, run the launch to bring up the robot and MoveIt planning:
```bash
ros2 launch arm_pkg frida_moveit_config.launch.py robot_ip:=192.168.31.180
```

### Motion Planning
Check that the motion planning interface is running:
```bash
ros2 run frida_motion_planning motion_planning_server.py
```

### Pick and Place
Most manipulation functions are enclosed within a manipulation server that handles tasks involving moving and interacting with objects. To run the manipulation server, use the following command:
```bash
ros2 launch pick_and_place pick_and_place.launch.py
```

### Examples
Examples to move the robot, edit collision objects, plan trajectories and using task-specific functions are available in most packages.