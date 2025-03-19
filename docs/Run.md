# Run through docker
To run or test the modules it is necessary to have Ubuntu 22.04. Alternatively, you can use the docker setup provided in this repository.

## Vision
To make a container and open a shell for developing or testing the vision package, use the script `docker/vision/run.sh`. This can be accessed from the general script `./run.sh` in the root directory by passing the argument vision. This will first build the base image according to your system (cpu, cuda or jetson) as well as the image for the vision module and then run the container.

*Note: the cuda docker file for vision hasn't been tested. Please report any issues.*

In root directory (home2), run:
```bash
./run.sh vision
```

If the script is not executable, run:
```bash
chmod +x run.sh
```

If the camera is not available run:
```bash
sudo chmod 666 /dev/video0
```