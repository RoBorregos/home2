# HRI
To run or test the modules it is necessary to have Ubuntu 22.04. Alternatively, you can use the docker setup provided in this repository.

## Docker setup
Run the setup script in `docker/hri/setup.bash`. This should be runned at least once per machine.

### Jetson Orin

In `docker/hri/` run:
```bash
docker compose up
```

Note: it is recommended to run the compose attached to see the logs of the services (e.g. stt, ollama, etc.)

### Laptop

For gpu or cpu, in `docker/hri/`, run:
```bash
docker compose -f docker-compose-cpu.yml up
```

# Running the hri module
Once in the docker workspace or using ROS2 in the home2 directory, run the following commands:

### Download llm models

Run the script found at `hri/packages/nlp/assets/download-model.sh` once. This will download the models, which will be mounted in the docker container. In addition, you may need to pull the deepseek model inside the container. To do this, run the following commands:

```bash
docker exec -it <container_id> bash # enter the container
ollama run deepseek-r1:7b # pull the model
```

### Build
```bash
colcon build --symlink-install --packages-select task_manager frida_interfaces frida_constants speech nlp embeddings

# Source the workspace
source ~/.bashrc
```

### Launch the module
```bash
ros2 launch speech hri_launch.py
```

# Additional Information

For more detailed information, see HRI's [README.md](../../../hri/README.md).