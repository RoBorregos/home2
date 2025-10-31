# Docker organization

This directory contains the Dockerfiles and scripts to build the Docker images for the different components of the project.

## Naming conventions for images and containers

Containers:

```bash
home2-<area>-<container_type>
```

Examples:

```bash
home2-hri-cpu
home2-hri-cuda
```

Images:

```bash
roborregos/home2:<area>-<image_type>
```

Examples:

```bash
roborregos/home2:hri-cpu
roborregos/home2:hri-cuda
```

Exceptions:

- Base images:

```bash
roborregos/home2:cpu_base
roborregos/home2:cuda_base
[l4t]
```

## Docker base images

The base images can be built by using the following docker compose files:

- cpu.yaml: Builds the base image for the CPU version of the project.
- cuda.yaml: Builds the base image for the GPU version of the project.

The images may be built independently by running the following commands:

```bash
docker compose -f docker/cpu.yaml build
docker compose -f docker/cuda.yaml build
```

The base images must be built before running areas' docker compose files.

## Using Docker Compose files and containers

```bash
# pwd -> home2/docker/hri
docker compose -f devices.yaml up # Create containers

docker compose -f devices.yaml down # Remove containers

docker compose -f devices.yaml stop # Stop containers

# Enter a container
docker exec -it <container_name> bash
```

## Dockerfile.ROS

The `Dockerfile.ROS` file is a template that can be used to install ROS humble in a plain docker image. This is used to build the base image for both `cpu.yaml` and `cuda.yaml`.

## Docker Hub Integration (Orin/Jetson Images)

For building and pushing Jetson Orin/L4T images to Docker Hub:

- **Quick Reference**: See [QUICK_REFERENCE.md](./QUICK_REFERENCE.md) for common commands
- **Full Documentation**: See [DOCKER_HUB_GUIDE.md](./DOCKER_HUB_GUIDE.md) for complete guide

### Quick Start

```bash
# Build and push all Orin images (using scripts)
./docker/push-orin-images.sh v1.0.0

# Or using Makefile
cd docker/
make deploy VERSION=v1.0.0

# Pull images
./docker/pull-orin-images.sh v1.0.0
```

### Available Scripts

- `push-orin-images.sh` - Build and push all Jetson images to Docker Hub
- `pull-orin-images.sh` - Pull all Jetson images from Docker Hub
- `list-orin-images.sh` - List available local Jetson images
- `Makefile` - Build automation with make commands

### GitHub Actions

The repository includes automated CI/CD via GitHub Actions (`.github/workflows/docker-orin.yml`). Images are automatically built and pushed when:

- Code is pushed to `main` or `stable` branches
- A new release is created
- Workflow is manually triggered

```

```
