ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

RUN mkdir -p /workspace/requirements

# Requirements
COPY task_manager/requirements /workspace/requirements
WORKDIR /workspace/requirements

RUN pip install -r requirements.txt

WORKDIR /workspace
# 

# CV-bridge
RUN apt update 
RUN apt install -y ros-humble-nav2-msgs
