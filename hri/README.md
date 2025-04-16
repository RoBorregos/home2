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

Run the script `setup.bash` located in `home2/docker/hri` to setup the configuration for docker.

In addition, the following files are required:
- `docker/hri/.env`: Contains the environment variables for the docker compose files. `docker/hri/.env.example` contains examples of the required variables.


## Using with docker

```bash
# Build base image
# pwd -> home2/docker
docker compose -f cuda.yaml build
# or -> docker compose -f cpu.yaml build

# Build and run HRI containers
# pwd -> home2/docker/hri
docker compose up

# Enter the container (this container has the ros 2 environment)
docker exec -it home2-hri-cuda-devices bash

```

## Running the project

Most of the final commands will be executed using the docker compose file.

However, some testing commands are the following:

```bash
# Launch HRI (includes speech, and nlp)
ros2 launch speech hri_launch.py

# Speech (Remember to start the stt docker before, this is done automatically if running the hri docker compose file)
ros2 launch speech devices_launch.py

# Say something
ros2 service call /hri/speech/speak frida_interfaces/srv/Speak "{text: \"Go to the kitchen and grab cookies\"}"

# Extract data from a sentence
ros2 service call /hri/nlp/data_extractor frida_interfaces/srv/ExtractInfo "{full_text: 'Hello, my name is Oscar', data: 'name'}"

# Is positive
ros2 service call /hri/nlp/is_positive frida_interfaces/srv/IsPositive "{text: 'I confirm'}"
ros2 service call /hri/speech/STT frida_interfaces/srv/STT {}

# NLP
ros2 launch nlp nlp_launch.py

ros2 topic pub /speech/raw_command std_msgs/msg/String "data: Go to the kitchen and grab cookies" --once
```

## Other useful commands

Source the environment (this is automatically done in the .bashrc)
```bash
source /workspace/install/setup.bash
```

Build the hri packages (this is automatically done in `hri-ros.yaml` docker compose file)
```bash
colcon build --symlink-install --packages-select task_manager frida_interfaces frida_constants speech nlp embeddings
```

Enable file permissions for the current user, this is useful if there is a mismatch between the user in the container and the user in the host.
```bash
# pwd -> home2
sudo chown -R $(id -u):$(id -g) .
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

## Setup speech default sink and source

Sinks and sources are the audio devices that pulseaudio uses to play and record audio. Setting the default sink and source is useful to make sure that the audio is played and recorded from the correct device.

```bash 
# Set default sink
nano ~/.config/pulse/default.pa
# Add the following line

# Respeaker 4 mic array
set-default-source alsa_input.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.multichannel-input

# Frida's speaker
set-default-sink alsa_output.usb-GeneralPlus_USB_Audio_Device-00.analog-stereo

# Restart pulseaudio (add to .bashrc)
pulseaudio -k && pulseaudio --start
```

### Debug speech devices

Sinks (Speakers)

```bash
# See default sink
pactl info | grep "Default Sink"
# Set default sink
pactl set-default-sink <index>
# List all sinks
pactl list short sinks
```

Sources (Microphones)

```bash
# See default source
pactl info | grep "Default Source"
# Set default source
pactl set-default-source <index>
# List all source
pactl list short sources
```

### Speaker

If the speaker isn't loud, make sure to increase the volume level in the device that controlls the speaker.
```bash
amixer -D pulse sset Master 100%
```

## Download openwakeword base model

```
python3
import openwakeword
openwakeword.utils.download_models()
```