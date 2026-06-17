# Tech Stack

This project leverages modern robotics and containerization technologies to provide a modular and scalable platform for FRIDA.

## Core Framework
- **ROS 2 (Robot Operating System 2)**: The primary framework for node communication, service management, and action orchestration.

## Programming Languages
- **Python**: Primarily used for high-level logic, HRI, and vision-related scripts.
- **C++**: Used for performance-critical components in navigation, manipulation, and core interfaces.

## Middleware & Communication
- **Cyclone DDS**: The default Data Distribution Service for ROS 2 communication.
- **Protocol Buffers (Proto)**: Used for some internal communication between microservices, particularly in the HRI area.

## Infrastructure
- **Docker & Docker Compose**: Used to containerize all modules, ensuring portability between development machines and the robot's hardware (e.g., Jetson Orin).
- **NVIDIA Container Toolkit**: Enables GPU acceleration within Docker for vision and deep learning tasks.

## Targeted Platforms
- **CPU**: Standard x86 development environments.
- **CUDA**: NVIDIA GPU-enabled systems (e.g., Desktop PCs with RTX GPUs).
- **L4T (Linux for Tegra)**: Specifically for NVIDIA Jetson platforms (e.g., Orin, Xavier).
