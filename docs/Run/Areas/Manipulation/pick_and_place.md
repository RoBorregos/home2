# Running Pick and Place
This example shows how to run a pick and place on our robot. Updated as of April 11, 2025.

## Launching the Robot

### Simulation
Launch the robot with our simulation environment on gazebo and loading gripper and 3D camera:
```bash
ros2 launch frida_description moveit.launch.py load_zed:=true
```

### Real RObot
#### Robot interface
On the real robot, run the following command to launch the robot with MoveIt planning, while on the same network as the robot:
```bash
ros2 launch arm_pkg frida_moveit_config.launch.py
```
#### 3D Camera
To run the 3D camera, while on the device connected to it, run the following command:
```bash
**ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 publish_tf:=false
```

## Launching utilities
These represent core functionalities that are used within the pipeline.
### Downsample Point Cloud
To run the downsample for the pointcloud:
```bash
ros2 run perception_3d down_sample_pc
```
### Motion planning server
Check that the motion planning interface is running:
```bash
ros2 run frida_motion_planning motion_planning_server.py
```
### Object detector
To run the object detector, run the following command:
```bash
ros2 launch object_detector_2d object_detector_node.launch.py
```
For this, be sure that the topics on [vision constants](../../../../frida_constants/frida_constants/vision_constants.py) are set to your published image and depth topics. These may change if you are using simulation or the real robot.

## Launch Pick and Place main code
### Perception
To run the perception service (Cluster and plane remove/extraction):
```bash
ros2 launch perception_3d test.launch.py
```
### Pick and Place pipeline
To run the pick and place pipeline:
```bash
ros2 launch pick_and_place pick_and_place.launch.py
```

## Usage
Up until now, this has enabled all functionalities required for the pick and place to work. Calls to it can be done following the action ManipulationTask located in frida_interfaces. To trigger manually, you have two options:

1. **Using the clicked point from Rviz**
   - Open Rviz and select the "Clicked Point" option in the upper task bar.
   - Click on the object you want to pick and place.

2. **Using the manual input**
    - Open a terminal and run the following command:
    ```bash
    ros2 run pick_and_place keyboard_input.py
    ```
    - Follow the instructions in the terminal to select the object you want to pick and place.
    - Type -2 to refresh objects if the robot is not seeing any. Use rqt to ensure the robot is seeing the object you actually want to pick and place, the object_detector script will launch a visualization of the detections.
