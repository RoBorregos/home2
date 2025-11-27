# Docker architecture

This directory contains the Dockerfiles and scripts to build the Docker images for the different components of the project. We currently mantain images for 3 different environments: `cpu`, `cuda` and `l4t`.

## Naming conventions

### Containers

```bash
home2-<area>-<container_type>
```

Examples:

```bash
home2-hri-cpu
home2-manipulation-cuda
home2-navigation-l4t
```

### Images

```bash
roborregos/home2:<area>-<image_type>
```

Examples:

```bash
roborregos/home2:vision-cpu
roborregos/home2:manipulation-cuda
```

Exceptions:

- Base images:

```bash
roborregos/home2:cpu_base
roborregos/home2:cuda_base
roborregos/home2:l4t_base
```

## Base images

These images are based on `ubuntu 22.04` and contain the basic setup for all areas using `ROS`. There is one for each environment ensuring the best performance. The right image is detected and built upon running `run.sh` using the `Dockerfiles` in this directory.

## Areas

Each directory contains their individual `Dockerfiles`, `docker compose yamls`, `run.sh` and other area specific files such as entrypoint scripts. Each area creates and mounts 3 directories: `build`, `install` and `log`. These are used to persist ROS2 data between containers, avoiding building packages each time a new container is created.

### Frida interfaces cache

Used to build and share via docker volumes 2 ROS packages shared by all areas: `frida_interfaces` and `frida_constants`. **This area doesn't have a `run.sh`**, instead it's ran:

- Automatically when root `run.sh` detects the packages haven't been built.
- Deterministically by executing root `run.sh` with the argument `frida_interfaces`. This removes the container after building.

### Human Robot Interaction and Vision

These areas manage a microservice architecture, opting out from `ROS` and communicating containers via `gRPC`, a high-performance framework for creating remote procedure calls.

### Manipulation and Navigation

These areas have a high `ROS2` dependency and work with only one centralized container.

### Integration

A centralized container for running tasks from the `task_manager`.
