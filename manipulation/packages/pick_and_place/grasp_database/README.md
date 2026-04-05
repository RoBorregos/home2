# Grasp Database

Pre-computed grasps for known objects. Eliminates GPD from the runtime pick pipeline.

## How to add a new object

### Step 1: Capture the model

**Option A -- With the robot (live capture):**
```bash
# Make sure the manipulation stack is running and the robot is in table_stare
# Place the object centered in front of the robot
ros2 run pick_and_place capture_object --ros-args -p name:=orange -p mode:=live
```

**Option B -- From an existing file (Polycam, other scanner):**
```bash
ros2 run pick_and_place capture_object --ros-args -p name:=orange -p mode:=file -p input:=/path/to/scan.ply
```

### Step 2: Generate grasps
```bash
# The manipulation stack must be running (GPD service needs to be available)
ros2 run pick_and_place generate_grasps --ros-args -p name:=orange
```
Wait ~10-30s for GPD to process. The script handles the GPD service restart automatically.

### Step 3: Verify
```bash
cat grasp_database/objects/orange/grasps.yaml
```
You should see 20-50 grasps with positions, orientations, and scores.

### Step 4: Test
Run a pick of the object. The logs should show:
```
Database grasp attempt: object=orange, grasps_found=10, time=2ms
```
If it says "falling back to GPD", the database grasps all failed MoveIt planning.

## Directory structure
```
grasp_database/
  objects/
    {object_name}/
      model.pcd         -- Point cloud of the object
      metadata.yaml     -- Centroid, viewpoint, capture date
      grasps.yaml       -- Pre-computed grasps (relative to centroid)
  database.yaml         -- Index of all objects
```

## Troubleshooting

- **"Object not in database"**: Run capture + generate for that object
- **"grasps_found=0"**: The object was detected at a position where all stored grasps are unreachable. Try recapturing with the object in a different position.
- **Database grasps fail but GPD works**: The stored grasps may not cover the current approach angle. Recapture from a different robot pose.
- **GPD service not available during generate**: Make sure the manipulation stack is running (`./run.sh manipulation --gpsr`)
