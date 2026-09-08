# FRIDA Architecture

The software architecture for **FRIDA** (Friendly Robotics Interactive Domestic Assistant) is built on **ROS 2** and designed with modularity in mind. The system is divided into several "Areas," each representing a specific robotic capability.

## Core Areas

Each area typically corresponds to a set of ROS 2 packages and containers:

- **Vision**: Handles image processing, object detection, and perception tasks.
- **Human-Robot Interaction (HRI)**: Manages communication with users through voice, visual interfaces, and social behaviors.
- **Navigation**: Responsible for localization, mapping, and path planning.
- **Manipulation**: Controls the robotic arms and grippers for tasks like picking and placing objects.
- **Integration**: Orchestrates the interaction between multiple areas to fulfill complex service robot challenges.

## Task Manager — Central Orchestrator

`task_manager/` is not just another area — it's the piece that coordinates the others. It hosts:

- **Task managers per competition challenge**: GPSR, restaurant, laundry, HRIC, pick & place.
- **Subtask managers**: reusable building blocks shared across challenges.
- **Manual test scripts** (`task_manager/scripts/test/test_*.py`), run individually via `ros2 run`, not as an automated suite.

Where Vision, HRI, Navigation, and Manipulation expose individual capabilities, `task_manager` is what sequences them to actually complete a RoboCup @Home challenge end to end.

## Shared Packages

Two packages are used across multiple areas rather than belonging to a single one:

- **`frida_constants/`**: a Python + CMake package holding constants and enums shared across areas, so values don't get duplicated or drift between packages.
- **`frida_interfaces/`**: custom `.msg` / `.srv` / `.action` definitions, organized per area. Always prefer these over ad-hoc message types for robot-specific interactions — see `docs/interfaces.md` for what's already defined.

## Third-Party & Forked Code (Git Submodules)

Several packages under `manipulation/` and `navigation/` are git submodules pointing to external or forked repositories rather than code owned by the team, including `gpd`, `pymoveit2`, `xarm_ros2`, `mujoco_ros2_control`, `vamp`, `sllidar_ros2`, `ira_laser_tools`, and `PlayStation-JoyInterface-ROS2`. This code follows upstream conventions, not ours — it's excluded from repo-wide linting for that reason, and shouldn't be refactored to match internal style without checking with maintainers first.

## `specs/`

An empty `specs/` directory exists at the repo root. It's presumably reserved for a future `/spec → /build` workflow (e.g. for AI coding agents) but currently has no content — don't assume conventions here yet.

## Execution Model

The project utilizes **Docker** and **Docker Compose** to ensure a consistent environment across different hardware platforms (CPU, CUDA, L4T/Jetson).

### The `run.sh` Script

A central `run.sh` script is used to manage the lifecycle of these containers. It automatically detects the environment and spins up the necessary services.

```bash
# Example: Run the vision area
./run.sh vision

# Example: Run a competition task (GPSR)
./run.sh --gpsr
```

Other supported challenge flags include `--restaurant`, `--ppc`, `--hric`, `--dlc`, and `--finals`.

### CI

`.github/workflows/ros2-build.yml` runs `colcon build --symlink-install` inside a `ros:humble` image, checking out submodules first. **It does not run automated tests** — see the Testing section in `coding_standards.md`. Other workflows handle submodule checks (`check-submodules.yml`), Docker image publishing (`docker-publish.yml`), and docs generation (`doxygen-gh-pages.yml`).

## Communication

Communication between nodes is handled by the **Cyclone DDS** middleware, which is configured via `scripts/setup_cyclonedds.sh` to allow seamless multi-host or host-container interaction.
