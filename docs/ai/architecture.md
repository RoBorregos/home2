# FRIDA Architecture

The software architecture for **FRIDA** (Friendly Robotics Interactive Domestic Assistant) is built on **ROS 2** and designed with modularity in mind. The system is divided into several "Areas," each representing a specific robotic capability.

## Core Areas

Each area typically corresponds to a set of ROS 2 packages and containers:

- **Vision**: Handles image processing, object detection, and perception tasks.
- **Human-Robot Interaction (HRI)**: Manages communication with users through voice, visual interfaces, and social behaviors.
- **Navigation**: Responsible for localization, mapping, and path planning.
- **Manipulation**: Controls the robotic arms and grippers for tasks like picking and placing objects.
- **Integration**: Orchestrates the interaction between multiple areas to fulfill complex service robot challenges.

## Execution Model

The project utilizes **Docker** and **Docker Compose** to ensure a consistent environment across different hardware platforms (CPU, CUDA, L4T).

### The `run.sh` Script

A central `run.sh` script is used to manage the lifecycle of these containers. It automatically detects the environment and spins up the necessary services.

```bash
# Example: Run the vision area
./run.sh vision

# Example: Run a competition task (GPSR)
./run.sh --gpsr
```

## Communication

Communication between nodes is handled by the **Cyclone DDS** middleware, which is configured via `scripts/setup_cyclonedds.sh` to allow seamless multi-host or host-container interaction.
