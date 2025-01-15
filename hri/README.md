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

Run the script `setup.bash` located in `home2/docker/hri` to setup the configuration for docker. The script provides additional setup instructions.

In addition, the following files are required:
- `docker/hri/.env`: Contains the environment variables for the docker compose files. `docker/hri/.env.example` contains examples of the required variables.


## Using with docker

```bash
# Build base image
# pwd -> home2/docker
docker compose -f cuda.yaml build
# or -> docker compose -f cpu.yaml build

# Use devices compose (for audio I/O)
# pwd -> home2/docker/hri
docker compose -f devices.yaml up

# Build packages
## Enter the container
docker exec -it home2-hri-cuda-devices bash

# Enable non-root ownership of the workspace
# pwd -> /workspace
sudo chown -R $(id -u):$(id -g) .

# pwd -> /workspace
colcon build --symlink-install --packages-select frida_interfaces speech nlp
source install/setup.bash
```

## Running the project

Most of the final commands will be executed using the docker compose file.

However, some testing commands are the following:

```bash
# Speech (Remember to start the whisper docker before)
ros2 launch speech devices_launch.py

ros2 topic pub /speech/speak_now --once std_msgs/msg/String "data: 'Go to the kitchen and grab cookies'"

# NLP
ros2 launch nlp nlp_launch.py

ros2 topic pub /speech/raw_command std_msgs/msg/String "data: Go to the kitchen and grab cookies" --once
```

## Speech pipeline

### AudioCapturer.py

Captures raw audio in chunks and publishes it.

- publish -> rawAudioChunk

### KWS.py

Uses porcupine to detect kew words sush as "Frida".

- subscribe -> rawAudioChunk
- publish -> keyword_detected

### UsefulAudio.py

Uses silero VAD to identify speech in raw audio and publish it to UsefulAudio.

- subscribe
    -> rawAudioChunk
    | saying
    | keyword_detected
- publish
    -> UsefulAudio
    | AudioState
    | colorInstruction
    | /ReSpeaker/light

### Hear.py

Takes UsefulAudio, performs STT with gRPC servers and publishes it.

- subscribe -> UsefulAudio
- publish -> /speech/transcription