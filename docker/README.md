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

## Dockerfile.ROS

The `Dockerfile.ROS` file is a template that can be used to install ROS humble in a plain docker image. This is used to build the base image for both `cpu.yaml` and `cuda.yaml`.
