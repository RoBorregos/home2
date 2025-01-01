# HRI

Tree structure

```bash
home2
│
│frida_interfaces # Contains the interfaces for the project
├── hri
│   ├── msg
│   └── srv
│
│hri
├── packages # Contains packages for the project
│   ├── nlp
│   └── speech
│       ├── CMakeLists.txt
│       ├── launch
│       ├── package.xml
│       ├── scripts
│       │   └── say.py # Allows robot to speak
│       └── speech
│           ├── __init__.py
│           ├── speech_api_utils.py # Contains util functions for speech
│           └── wav_utils.py # Contians util functions for wav files
├── README.md # This file
└── requirements # Python dependencies for the project
    ├── nlp.txt
    └── speech.txt
```

## Setup with docker

```bash
# Build base image
# pwd -> home2/docker
docker compose -f cuda.yaml up

# Use devices compose (for audio I/O)
# pwd -> home2/docker/hri
docker compose -f devices.yaml up

# Build packages
## Enter the container
docker exec -it home2-hri-cuda-devices bash

# Enable non-root ownership of the workspace
sudo chown -R $(id -u):$(id -g) .

# pwd -> /workspace
colcon build --symlink-install  --packages-select frida_interfaces speech

```

## Running the project

Most of the final commands will be executed using the docker compose file.

However, some testing commands are the following:

```bash
ros2 run speech say.py

ros2 topic pub /speech/speak_now --once std_msgs/msg/String "data: 'Go to the kitchen and grab cookies'"
```
