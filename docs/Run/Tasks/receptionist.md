# Receptionist

## Vision
Run container. From the root directory (home2), run:
```bash
./run.sh vision
```

Build and source
```bash
colcon build --packages-up-to vision_general moondream_run
source install/setup.bash
```

Launch
```bash
ros2 launch vision_general receptionist_launch.py
```