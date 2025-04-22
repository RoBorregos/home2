# 🗺️ Map Tagger Application

**Manual map contextualization tool** using RViz and ROS 2.

This tool allows you to tag a map interactively using RViz’s **2D Goal Pose** tool.  
You can define areas for **rooms**, **paths**, **objects of interest**, and **global hallways**, then export or load them via YAML.

---

## 🧠 Global App States

```
1. Add room entrance(s)  
2. Add room area  
3. Add room path  
4. Add object of interest  
5. Save current map to YAML  
6. Load map from YAML  
7. Add global hallway/external path  
8. State current room number  
9. State current global path number  
0. Do nothing / wait  
-1. Exit  
```

---

## 🚀 How to Use

### 1. Build the packages

```bash
colcon build --packages-select map_context_tests map_context_interfaces
```

### 2. Launch the tool

```bash
ros2 launch map_context_tests map_tagger.launch.py
```

This will:
- Open RViz with the appropriate configuration
- Launch a terminal with the map tagging menu

Alternatively, you can run them separately:

```bash
ros2 run map_context_tests map_tagger
rviz2
```

---

## 📂 Example

To view an example map:

- Select option `6`
- Enter the filename: `home_example5`

---

## 📝 Notes

- The order of the 2D goal poses **must be selected counter clockwise**, or the side with color will be looking "down" (idk why)  
- if it breaks, restart / relaunch

---

## ⚠️ Pending Tasks

- [ ] FRIDA-specific pose saver     
- [ ] Ensure every input path leads back to the prompt menu  
- [ ] Calculate dimensions of rooms/objects  
- [ ] Fix color/normal facing bug with area polygons  
- [ ] General UX and stability improvements  
