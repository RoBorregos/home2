# HRI

In order to run and setup this area specific flags were added which abstract the complexity of performing both simple and complex tasks.

## Flags

Adding multiple flags in the same command is supported. Remember that to run a specific task the task's flag must be the first one.

```bash
./run.sh hri --gpsr
```

Launches HRI with the given task. The PyQt display UI ([hri/packages/display/scripts/display_ui.py](../../../hri/packages/display/scripts/display_ui.py)) starts automatically inside the container and shows the task's specific view (via the `display_task` arg forwarded to `hri_launch.py`) — no separate "open display" step needed, the window renders directly on the forwarded X11 `DISPLAY`.

```bash
./run.sh hri --gpsr --backup
```

Same as above, but launches the legacy Next.js display (`display_launch_backup.py`, rosbridge + web_video_server + a kiosk browser window) instead of the default PyQt UI. Kept as a fallback, not deleted. `./run.sh display --backup` runs it standalone the same way.

```bash
./run.sh hri --download-model
```

Pulls RoBorregos' command interpreter LLM from hugging face and other ollama models. See `docker/hri/scripts/download-model.sh`.

```bash
./run.sh hri --regenerate-db
```

Uses current json files from `hri/packages/embeddings/embeddings/dataframes/` and a navigation service to retrive information and generate SQL files which are saved in `/docker/hri/sql_dumps/`. Then it replaces the information in the postgres database with these scripts. See `docker/hri/scripts/regenerate_db.sh`.

```bash
./run.sh hri --build-proto
```

Generates gRPC files for the microservices using the .proto files located in `hri/proto_interfaces/proto_interfaces/`.

## Running specific containers

You may want to only launch some of the services. For example, running the module without the LLM, etc. To do so, please check the docker-compose yaml files in the `docker/hri/compose/` folder and comment out the services you do not want to run.

For more detailed information, see HRI's [README.md](../../../hri/README.md).
