# Expo demo Feb 13, 2025

Note: run in branch `hri_manager`

## Running HRI

This runs a basic Q&A (llm wrapper), with keyword spotting.

Note: requires wifi connection for llm (local option could be set up in `hri/packages/nlp/config/command_interpreter.yaml`, but ollama setup is currently not available in this repo)

For the question answering, you can add more context in `hri/packages/nlp/nlp/assets/prompts.py`. For better word recognition, you can add hotwords in `hri/packages/speech/scripts/stt/Faster-whisper.py` (line 43)

In laptop (doesn't need gpu)

Edit `home2/hri/packages/speech/config/hear.yaml` by adding your laptop's IP address in the `STT_SERVER_IP` field.

```bash
# pwd -> home2/docker/hri
docker compose -f stt.yaml up
```

In jetson
```bash
# Launch mic and speaker (make sure they are connected and the speaker is turned on)
# pwd -> /mnt/kingston/RoBorregos/home2/docker/hri
docker compose -f l4t-devices.yaml up

docker exec -it home2-hri-l4t-devices bash

# Launch gpsr task manager
ros2 run task_manager gpsr_task_manager.py 
```

