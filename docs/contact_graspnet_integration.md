# Contact-GraspNet Integration Documentation

This document summarizes the transition from GPD (Grasp Pose Detection) to Contact-GraspNet within the `home2` repository.

## 1. Motivation
The project previously relied on **GPD**, which is a geometric-based detector that:
- Struggles with cluttered environments.
- Has lower success rates for unseen objects.
- Is relatively slow on CPU-only or low-power GPU setups.

**AnyGrasp** was considered as an alternative but was rejected due to:
- Licensing restrictions (SDK-only, proprietary).
- Architecture mismatch (x86 binaries provided, while the target is aarch64 Orin).

**Contact-GraspNet** was chosen because:
- It provides 6-DoF grasp distribution from raw scene point clouds.
- It is significantly more robust in clutter.
- A PyTorch port is available, allowing for better optimization on NVIDIA Jetson (Orin).

## 2. Repository Structure & Artifacts

### Core Model (Submodule)
- **Path:** `manipulation/packages/contact_graspnet`
- **Source:** [elchun/contact_graspnet_pytorch](https://github.com/elchun/contact_graspnet_pytorch)
- **Role:** Contains the PyTorch implementation of the model and inference logic.

### ROS2 Wrapper
- **Path:** `manipulation/packages/contact_graspnet_ros`
- **Node:** `contact_graspnet_node`
- **Service:** `detect_grasps` (`frida_interfaces/srv/GraspDetection`)
- **Launch:** `ros2 launch contact_graspnet_ros contact_graspnet.launch.py`

### Docker & Environment
- **Requirements:** Updated `manipulation/requirements.txt` with `torch`, `open3d`, `trimesh`, `scipy`, etc.
- **Setup Script:** `docker/manipulation/setup_contact_graspnet.sh`. This script handles submodule dependencies and compiles CUDA extensions for PointNet++.
- **Configuration:** Updated `Dockerfile.cuda`, `Dockerfile.l4t`, and `docker-compose-*.yaml` to include and mount the setup script.

## 3. Current Implementation Status (April 2026)
- [x] Create feature branch: `feature/contact-graspnet-integration`.
- [x] Add submodule and create ROS2 wrapper.
- [x] Update Docker and dependency configurations.
- [x] Implement ROS2 Service Handler (`PointCloud2` -> `PoseStamped[]`).
- [x] Successfully build package in local environment.
- [x] Debug node initialization (missing `trimesh` dependency fixed).
- [ ] **Pending:** Download/Verify pre-trained weights (`model.pt`) in `checkpoints/`.
- [ ] **Pending:** Final validation on NVIDIA Orin (L4T).

## 4. How to Test
1. **Build:** `colcon build --packages-select contact_graspnet_ros`
2. **Setup:** Run `/home/ros/setup_contact_graspnet.sh` inside the container to ensure all extensions are compiled.
3. **Launch:** `ros2 launch contact_graspnet_ros contact_graspnet.launch.py`
4. **Service Call:** 
   ```bash
   ros2 service call /detect_grasps frida_interfaces/srv/GraspDetection "{input_cloud: {header: {frame_id: 'base_link'}}}"
   ```

## 5. Technical Notes
- The node currently assumes `local_regions=False` to process the entire scene.
- The `matrix_to_quaternion` conversion is handled internally in the Python node to avoid extra dependencies like `tf_transformations` if not present.
- Integration with the `pick_and_place` package is seamless as long as the service name remains `detect_grasps`.
