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

## Self-Diagnosis & Supervised Self-Healing

The Integration area (task manager) can detect, diagnose, and recover from mid-task
subsystem failures instead of freezing until timeout.

- **Health engine** (`status/`): `health_monitor.py` exposes the live TUI's
  node/DDS/container checks as a programmatic API; `log_collector.py` +
  `log_parser.py` isolate known error classes (segfault, DDS, tf2, build) from
  `docker logs` and `~/.ros/log/`.
- **Diagnosis oracle** (`task_manager/task_manager/diagnosis/`): a *standalone*
  local-Ollama oracle (`oracle.py`) — deliberately independent of the HRI nlp
  service, since HRI may be the failure — with local RAG over `docs/ai/`
  (`knowledge_base.py`). It returns `{razon_falla, accion_sugerida}` where the
  action is drawn from a **closed** catalog (`actions.py`).
- **Supervised orchestrator** (`orchestrator.py`): proposes the corrective action
  (`./run.sh <area> --recreate`, `colcon build`, DDS/serial reset) and, by
  default, waits for human confirmation before executing, then re-verifies health.
- **Health-gate** (`health_gate.py` + `task_manager/config/subtasks_catalog.json`):
  before each subtask the FSM checks that the subtask's `critical_nodes` are
  alive. In active mode (`HOME2_AUTO_HEAL=1`) an unhealthy subtask routes through
  the task manager's `MAINTENANCE` state (stop actuators → diagnose → heal →
  re-verify → retry or dynamic re-plan).

**Maintenance note:** `subtasks_catalog.json` maps each skill (action in
`task_manager/.../baml_src/robot_commands.baml`) to its critical ROS 2 nodes and
must stay in sync with both that BAML catalog and `status/configs/*_nodes.cfg`.
