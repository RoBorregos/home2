# Vision
To run or test the modules it is necessary to have Ubuntu 22.04. Alternatively, you can use the docker setup provided in this repository.

## Docker setup
To make a container and open a shell for developing or testing the vision package, use the script `docker/vision/run.sh`. This can be accessed from the general script `./run.sh` in the root directory by passing the argument vision. This will first build the base image according to your system (cpu, cuda or jetson) as well as the image for the vision module and then run the container.

In root directory (home2), run:
```bash
./run.sh vision
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
Run the following command for each package or packages that should be built:

```bash
colcon build --packages-up-to vision_general 
```

Then source:

```bash
source install/setup.bash
```

### Run
Run a node using the following command:

```bash
ros2 run <package> <node_name>
ros2 run vision_general zed_simulator.py
```

# Structure
Vision is divided into the following packages:
- vision_general
- moondream_run
- object_detector_2d
- object_detection_handler

# Camera
To use de zed camera run the following command in orin:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 publish_tf:=false
```

If no zed camera is available you can run the zed simulator with an alternative camera with the following command:


```bash
ros2 run vision_general zed_simulator.py --ros-args -p video_id:=1
```

The video_id parameter is the id of the camera. To test camera ids and find the id or simply test the camera, run the following command:

```bash
python3 vision/packages/vision_general/Utils/camera_test.py <video_id>
python3 vision/packages/vision_general/Utils/camera_test.py 2
```

Test different ids by passing the argument. Default is 0.
