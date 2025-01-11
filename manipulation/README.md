# HRI

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

To make a container and open a shell for developing or testing the vision package, use the script `docker/manipulation/run.sh`. This can be accessed from the general script `./run.sh` in the root directory by passing the argument manipulation. This will first build the base image according to your system (cpu only or cuda enabled) as well as the full manipulation image and container.

*Note: Please report any issues to A00833160@tec.mx*

In root directory (home2), run:
```bash
./run manipulation
```

## Running the project

Most of the final commands will be executed using the docker compose file. *On development, you can use the docker run command to test the container.*
