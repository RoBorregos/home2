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
colcon build --packages-up-to vision_general moondream_run
```

Then source:

```bash
source install/setup.bash
```

### Run
Run a node using the following command:

```bash
ros2 run <package> <node_name>
```

# Structure
Vision is divided into the following packages:
- moondream_run
- vision_general
- object_detector_2d

