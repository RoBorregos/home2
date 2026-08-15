# Vision

> Full area documentation lives in [`vision/README.md`](../../../vision/README.md).

To run or test the modules it is necessary to have Ubuntu 22.04. Alternatively, you can use the docker setup provided in this repository.

## Docker setup

To make a container and open a shell for developing or testing the vision packages, use the script `docker/vision/run.sh`. This can be accessed from the general script `./run.sh` in the root directory by passing the argument vision. This will first build the base image according to your system (cuda or jetson) as well as the image for the vision module and then run the container.

In root directory (home2), run:
```bash
./run.sh vision
```

If the script is not executable, run:
```bash
chmod +x run.sh
```

If the camera is not available run the following command before running the `./run.sh vision` command:
```bash
sudo chmod 666 /dev/video0
```

# Running the vision module

Each competition task has its own flag, which builds the needed packages and starts the matching launch file:

```bash
./run.sh vision --hric
./run.sh vision --gpsr
./run.sh vision --ppc
./run.sh vision --dlc
./run.sh vision --restaurant
```

Add `--build` to compile before launching, or `-d` to run detached.

### Build

Once in the docker workspace, run the following command for the package or packages that should be built:

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
- object_detector_2d
- moondream_run

# Camera

To use the zed camera run the following command in orin:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 publish_tf:=false
```

If no zed camera is available you can run the zed simulator with an alternative camera with the following command:

```bash
ros2 run vision_general zed_simulator.py --ros-args -p video_id:=1
```

The video_id parameter is the id of the camera (`/dev/videoN`). Test different ids by passing the argument. Default is 0.
