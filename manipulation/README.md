# Manipulation

Tree structure

```bash
home2
│
│frida_interfaces # Contains the interfaces for the project
├── manipulation
│   ├── msg
│   └── srv
│   └── action
│
│manipulation
├── packages # Contains packages for the project
│   ├── 
│   └──
├── README.md # This file
└── requirements
```

## Setup with docker

### Requirements
- Docker Engine
- Nvidia Container Toolkit (for CUDA enabled images)

To make a container and open a shell for developing or testing the vision package, use the script `docker/manipulation/run.sh`. This can be accessed from the general script `./run.sh` in the root directory by passing the argument manipulation. This will first build the base image according to your system (cpu only or cuda enabled) as well as the full manipulation image and container.

*Note: Please report any issues to A00833160@tec.mx*
*Only CUDA images have been tested*

Clone directory:
```bash
git clone https://github.com/RoBorregos/home2
```

In root directory (home2), run:
```bash
cd home2
./run manipulation
```

This will install ROS2 dependencies and mount your cloned repository for easy access to the code. Inside the container, you can run the following commands to build the project:

```bash
colcon build --symlink-install
```

If you need to rebuild the image or container, you can run the following commands:

```bash
./run.sh manipulation --rebuild
```

## Running the project

Most of the final commands will be executed using the docker compose file. *On development, you can use the docker run command to test the container.*
