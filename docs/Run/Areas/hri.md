# HRI
To run or test the modules it is necessary to have Ubuntu 22.04. Alternatively, you can use the docker setup provided in this repository.

## Docker setup
### Quick start using run.sh
In root directory (home2), run:
```bash
./run.sh hri
```

This will:
- Build the base image according to your system (cpu, cuda or jetson) as well as the image for the hri module.
- Run the device setup script.
- Download the needed LLM models.

To run a specific task, specify it with the flag `--task`:
```bash
./run.sh hri --receptionist
```

# Additional Information

You may want to only launch some of the services. For example, running the module without the LLM, etc. To do so, please check the docker-compose files in the `hri` folder, and disable the services you do not want to run.

For more detailed information, see HRI's [README.md](../../../hri/README.md).
