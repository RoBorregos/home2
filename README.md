# RoBorregos @Home

[![ROS Build](https://github.com/RoBorregos/home2/actions/workflows/ros2-build.yml/badge.svg)](https://github.com/RoBorregos/home2/actions/workflows/ros2-build.yml)
[![Pre-commit](https://github.com/RoBorregos/home2/actions/workflows/pre-commit.yml/badge.svg)](https://github.com/RoBorregos/home2/actions/workflows/pre-commit.yml)

ROS 2 repository by [RoBorregos](https://github.com/RoBorregos), the Robotics Representative Team of Tecnologico de Monterrey. This project focuses on developing a modular software architecture for FRIDA (Friendly Robotics Interactive Domestic Assistant), an autonomous service robot competing in the [RoboCup @Home](https://athome.robocup.org/) league. The system enables robots to perform everyday domestic tasks through integrated vision, navigation, manipulation, and human-robot interaction modules.

## Table of Contents

- [Getting Started](#getting-started)
- [Usage](#usage)
- [Cyclone DDS Setup](#cyclone-dds-setup)
- [Documentation](#documentation)
- [Video Demonstrations](#video-demonstrations)
- [Team Members](#team-members)
- [Past Contributors](#past-contributors)

## Getting Started

### Prerequisites

- [Docker](https://docs.docker.com/engine/install/)
- [Docker Compose](https://docs.docker.com/compose/install/)
- [Git](https://git-scm.com/downloads)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) (optional, for GPU support)

### Installation

```bash
git clone https://github.com/RoBorregos/home2.git --recursive
cd home2

# If you already cloned without --recursive
git submodule update --init --recursive
```

## Usage

The `run.sh` script automatically detects your environment (CPU, CUDA, or L4T) and manages the Docker containers.

```bash
# Show available commands and options
./run.sh --help

# Run a specific area
./run.sh vision
./run.sh hri
./run.sh navigation
./run.sh manipulation
./run.sh integration

# Run a competition task
./run.sh --gpsr
./run.sh --restaurant
```

### Available Flags

| Flag | Description |
| --- | --- |
| `--build` | Builds the ROS 2 packages inside the container |
| `--build-image` | Builds the Docker image for the specified area or task |
| `--recreate` | Forces the recreation of containers |
| `--open-display` | Opens the graphical interface for HRI or Vision |
| `--stop` | Stops the running containers |
| `--down` | Stops and removes all containers, networks, and volumes |

## Cyclone DDS Setup

To enable communication between multiple computers (e.g., robot and development machine), set up Cyclone DDS on each host computer.

### Bare Metal (Orin, direct install)

```bash
sudo bash setup_cyclonedds.sh <INTERFACE>
source ~/.bashrc
```

### Docker Setup (PC)

Run the host-only setup once to apply kernel buffer settings:

```bash
sudo bash setup_cyclonedds.sh --host-only <INTERFACE>
```

Then run your area normally — `run.sh` reads the configuration automatically:

```bash
./run.sh navigation
```

Replace `<INTERFACE>` with your network interface (e.g., `eno1`, `wlp2s0`). Find it with:

```bash
ip -br link show
```

### Revert to FastDDS

```bash
sudo bash setup_cyclonedds.sh --revert
source ~/.bashrc
```

For detailed configuration options, see the [CycloneDDS setup docs](docs/cyclonedds-setup.md).

## Documentation

- [Project Docs](https://athome.roborregos.com/)
- [API Reference](https://roborregos.github.io/home2/)
- [RoboCup @Home Rule Book](https://robocupathome.github.io/RuleBook/rulebook/master.pdf)

[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/RoBorregos/home2)

## Video Demonstrations

[![Explanatory video demonstration of task planning test using the command interpreter with the robot](https://img.youtube.com/vi/do1S1zfmMsA/0.jpg)](https://www.youtube.com/watch?v=do1S1zfmMsA)

[![Video demonstration of the GPSR task during the Mexican Robotics Tournament 2025](https://img.youtube.com/vi/0bMz6ESv6B8/0.jpg)](https://www.youtube.com/watch?v=0bMz6ESv6B8)

[![Video demonstration](https://img.youtube.com/vi/qUQNTDRBEKw/0.jpg)](https://www.youtube.com/watch?v=qUQNTDRBEKw)

## Team Members

| Name | GitHub | Role |
| --- | --- | --- |
| Oscar Arreola | [@Oscar-gg](https://github.com/Oscar-gg) | HRI, Integration |
| Gerardo Fregoso | [@GerardoFJ](https://github.com/GerardoFJ) | Navigation, Integration |
| Alejandra Coeto | [@Ale-Coeto](https://github.com/Ale-Coeto) | Vision, Integration |
| Danae Sanchez | [@DanaeSG](https://github.com/DanaeSG) | Vision, Navigation, Integration |
| Alejandro Gonzalez | [@AleGonzcamilla](https://github.com/AleGonzcamilla) | Mechanics, Manipulation |
| Emil Winkler | [@emilwinkp](https://github.com/emilwinkp) | Manipulation |
| Fernando Hernandez | [@Fernando94654](https://github.com/Fernando94654) | Vision, Manipulation |
| Camila Tite | [@CamilaTite26](https://github.com/CamilaTite26) | HRI |
| Daniela Herrera | [@DanHeGa](https://github.com/DanHeGa) | Vision |
| Gilberto Malagamba | [@GilMM27](https://github.com/GilMM27) | HRI, Integration |
| Jose Dominguez | [@JLDominguezM](https://github.com/JLDominguezM) | Manipulation, Integration |

## Past Contributors

| Name | GitHub | Role |
| --- | --- | --- |
| Adan Flores-Ramirez | [@afr2903](https://github.com/afr2903) | Research |
| David Vazquez | [@deivideich](https://github.com/Deivideich) | Manipulation |
| Emiliano Flores | [@EmilianoHFlores](https://github.com/EmilianoHFlores) | Manipulation, Vision, Integration |
| Ivan Romero | [@IvanRomero03](https://github.com/IvanRomero03) | Integration, HRI, Manipulation, Vision |
| Diego Hernandez | [@Diego-HC](https://github.com/Diego-HC) | Integration, HRI, Navigation |
