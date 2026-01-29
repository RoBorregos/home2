# FRIDA VAMP Bridge

This package serves as the integration core for **FRIDA** (xArm 6). it acts as the definitive bridge between the robot's physical description and the high-performance **VAMP** (Variational Automated Motion Planning) engine. It enables smooth trajectory planning in cluttered environments while ensuring physical integrity through an optimized collision system.

---

## Prerequisites
Ensure the following are installed and configured:
* **FOAM**: For spheroidal mesh decomposition.
* **Cricket**: For C++ collision code generation.
* **VAMP**: The probabilistic motion planning engine.
* **Ubuntu 22.04+** & **Python 3.10+** (Tested on 3.13).

---

## Workflow Pipeline:
Follow these steps in order to reconstruct the planning model if the robot's URDF changes.

### Phase 1: Spherization with FOAM
VAMP does not process STL meshes directly for efficiency. FOAM approximates the metal's volume using optimal spheres.
```bash
# General command (adjust paths to your files)
foam frida_original.urdf frida_spherized.urdf
```

* Input: URDF with visual meshes.

* Output: URDF where <collision> tags contain sphere primitives.

### Phase 2: C++ Generation with Cricket
Cricket translates the URDF spheres into ultra-low latency C++ mathematical collision functions.

```bash
# Generate the C++ header
cricket frida_spherized.urdf -o frida_real.hh
```
* Important: Move the generated file to the VAMP implementation folder: mv frida_real.hh ~/roborregos/home_ws/src/manipulation/packages/vamp/src/impl/vamp/robots/

### Phase 3: The "Ghost Fix" (Materializing the Robot)
Cricket may generate empty collision blocks { }. Without this fix, the robot behaves like a "ghost," detecting collisions but failing to signal the planner to stop.

Run the repair script:

````bash
python3 scripts/fix_vamp.py
````
What does this script do?

* Scans the 414 collision blocks in the .hh file.

* Injects the return false; statement so the planner rejects invalid states.

* Cleans the file headers to prevent C++ compilation errors in struct definitions.

### Phase 4: Nuclear Compilation
Rebuild the C++ engine and update the Python bindings.
```bash
cd ~/roborregos/home_ws/src/manipulation/packages/vamp/build
rm -rf * && cmake ..
make -j$(nproc)

# Update the binary in the Python library
cp -v _core_ext.cpython-*.so ../src/vamp/_core/
```

### Usage and Execution
To test the system with a dynamic obstacle in PyBullet:

```bash
cd /src/manipulation/packages/vamp/scripts
python3 test_frida.py
```

## Technical Concepts Applied

### Configuration Space (C-Space)
The planner does not operate in Cartesian coordinates **(X, Y, Z)**, but in the joint space **C** ∈ **R⁶**. **VAMP** searches for a continuous path in this space that does not intersect collision regions (**C_obs**). Every point in **C** represents a unique set of angles for the 6 motors of the xArm.



### RRT-Connect (RRTC)
We use a variant of the **Rapidly-exploring Random Tree** algorithm called **RRT-Connect**. This expands two trees simultaneously—one from the starting pose and one from the goal pose—until they meet in the center. This approach is significantly more efficient for high-DOF (Degree of Freedom) manipulators like FRIDA compared to standard RRT.



### Trajectory Smoothing
* **Interpolation:** The raw points provided by the planner are subdivided (default: 50 steps) using linear interpolation to ensure fluid, jitter-free movement within PyBullet and on the physical hardware.
* **Simplify:** The system uses a "Shortcut" optimizer. After a path is found, it attempts to "straighten" the segments by connecting non-adjacent points if the straight line between them is collision-free. This makes FRIDA move in a more "human-like" and direct manner.

---

## Troubleshooting (FAQ)

* **Error: `expected unqualified-id before ‘return’`**
  * **Cause:** The `fix_vamp.py` script was too aggressive and injected code into C++ structure definitions.
  * **Solution:** Manually open the `.hh` file and remove the `return false;` statements from the first 100 lines (usually inside the `struct` definitions at the top).

* **Robot passes through itself (Ghosting)**
  * **Cause:** The binary was not compiled correctly, or the `.so` file was not copied to the library folder after applying the Phase 3 patch.
  * **Solution:** Repeat **Phase 4** (clean build and copy). Verify the fix by running a validation check on a known self-colliding pose.

* **VAMP cannot find `_core_ext`**
  * **Cause:** Python cannot locate the compiled C++ bindings.
  * **Solution:** Ensure your `PYTHONPATH` environment variable includes the path to `src/vamp/_core`.

---

> [!IMPORTANT]
> **Team Note:** This workflow is vital for all Manipulation tasks in the **TMR**. Any physical change to the robot, such as switching or modifying the gripper, **requires re-spherizing with FOAM** and regenerating the collision header. Otherwise, the end-effector will collide with objects or the robot base during autonomous tasks.