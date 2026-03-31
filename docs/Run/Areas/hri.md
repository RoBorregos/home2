# HRI

In order to run and setup this area specific flags were added which abstract the complexity of performing both simple and complex tasks.

## Flags

Adding multiple flags in the same command is supported. Remember that to run a specific task the task's flag must be the first one.

```bash
./run.sh hri --build-display
```

Downloads node-modules (dependencies) and builds the next.js web page inside a temporary `hri-ros` container.

```bash
./run.sh hri --open-display
```

Opens a new Firefox window in kiosk mode (fullscreen). If running with a task it will open the task's specific URL. See `docker/hri/scripts/open-display.bash`.

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
