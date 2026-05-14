# GEMINI.md - FRIDA Software Stack (home2)

This document provides instructional context and guidelines for the FRIDA (Friendly Robotics Interactive Domestic Assistant) software stack, developed by RoBorregos for RoboCup @Home.

## Project Overview

FRIDA is an autonomous service robot. The `home2` repository houses its modular software architecture, built on **ROS 2 Humble**. The system is designed to be highly portable and scalable through heavy use of Docker orchestration.

### Key Technologies
- **Middleware**: ROS 2 Humble.
- **Orchestration**: Docker & Docker Compose.
- **DDS**: Cyclone DDS (with Shared Memory support via RouDi/Iceoryx).
- **Languages**: Python (Task Management, HRI, Vision) and C++ (Interfaces, Navigation).
- **Hardware Support**: CPU-only, NVIDIA CUDA (Desktop GPUs), and NVIDIA L4T (Jetson Orin).

## Core Architecture

The project is organized into functional **Areas**:

- **Vision** (`/vision`): Object detection, tracking, and visual perception.
- **Navigation** (`/navigation`): Mapping, localization (SLAM), and autonomous movement.
- **Manipulation** (`/manipulation`): Robotic arm control and grasping.
- **HRI** (`/hri`): Human-Robot Interaction, including speech, face following, and UI.
- **Task Manager** (`/task_manager`): State machines and high-level logic for competition tasks.
- **Frida Interfaces** (`/frida_interfaces`): Centralized definitions for custom messages, services, and actions.
- **Frida Constants** (`/frida_constants`): Shared configuration data and static assets.

## Workflow & Commands

The `run.sh` script is the primary interface for managing the development environment. It automatically detects the hardware environment (CPU, CUDA, L4T).

### Common Commands

| Task | Command |
| --- | --- |
| **Start an Area** | `./run.sh <area>` (e.g., `vision`, `hri`, `navigation`, `manipulation`, `integration`) |
| **Start a Task** | `./run.sh --<task_flag>` (e.g., `--gpsr`, `--restaurant`, `--hric`) |
| **Build ROS Packages** | `./run.sh <area> --build` |
| **Build Docker Images** | `./run.sh <area> --build-image` |
| **Stop Environment** | `./run.sh --stop` (stop) or `./run.sh --down` (remove) |
| **Clean Workspace** | `./run.sh --clean` (removes build/install/log folders) |

### Network & Communication (Cyclone DDS)

Proper communication between containers or machines requires Cyclone DDS configuration:
- **Setup**: `sudo bash setup_cyclonedds.sh <NETWORK_INTERFACE>`
- **Revert**: `sudo bash setup_cyclonedds.sh --revert`

## Development Guidelines

1.  **Surgical Updates**: When modifying a specific area, ensure that changes are reflected in the corresponding Docker configuration if new dependencies are added.
2.  **Interface Consistency**: Always update `frida_interfaces` if new communication types are needed between areas.
3.  **Environment Awareness**: Code should ideally be agnostic to the underlying hardware (CPU/GPU) unless it's area-specific (e.g., Vision).
4.  **Testing**:
    - Use `run.sh <area> --build` to verify compilation.
    - Check node status with `status.sh` or within the container using `ros2 node list`.
5.  **Coding Style**:
    - Follow ROS 2 style guidelines.
    - Use `ruff` for Python linting (config in `ruff.toml`).
    - Pre-commit hooks are configured in `.pre-commit-config.yaml`.

## Directory Map (Condensed)

- `docker/`: Dockerfiles, compose files, and environment-specific configs (`cpu.yaml`, `cuda.yaml`, `l4t.yaml`).
- `docs/`: Technical documentation and setup guides.
- `frida_interfaces/`: ROS 2 message/service definitions.
- `task_manager/scripts/`: Implementation of competition routines.
- `lib.sh`: Common bash functions used by the auxiliary scripts.
