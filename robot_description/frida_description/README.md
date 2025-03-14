Command to run FRIDA simulation in Gazebo:

```python
ros2 launch frida_description moveit.launch.py add_gripper:=true add_realsense_d435i:=true
```

> [!WARNING]
> If the arm spawns incorrectly, please run again the simulation and check through the arm transform in the Rviz window
