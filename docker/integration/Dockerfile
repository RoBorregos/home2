ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

RUN mkdir -p /workspace/requirements

# Requirements
COPY task_manager/requirements /workspace/requirements
WORKDIR /workspace/requirements
RUN pip install --upgrade pip
RUN pip install -r requirements.txt

COPY ../../hri/requirements/postgres.txt /workspace/requirements/postgres.txt
RUN pip install -r postgres.txt

COPY task_manager/scripts/utils /workspace/utils
WORKDIR /workspace/utils
RUN baml-cli generate

WORKDIR /workspace

# CV-bridge
RUN apt update
RUN apt install -y ros-humble-nav2-msgs

# CycloneDDS
RUN apt-get update -qq && apt-get install -y -qq ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
COPY docker/cyclonedds_setup.sh /usr/local/bin/cyclonedds_setup.sh
RUN chmod +x /usr/local/bin/cyclonedds_setup.sh && /usr/local/bin/cyclonedds_setup.sh
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml
