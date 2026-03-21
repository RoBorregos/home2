ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

RUN mkdir -p /workspace/requirements

# Requirements
COPY task_manager/requirements /workspace/requirements
WORKDIR /workspace/requirements
RUN pip install --upgrade pip
RUN pip install -r requirements.txt

COPY hri/requirements/postgres.txt /workspace/requirements/postgres.txt
RUN pip install -r postgres.txt

COPY task_manager/scripts/utils /workspace/utils
WORKDIR /workspace/utils
RUN baml-cli generate

WORKDIR /workspace

COPY ./ /workspace/src
RUN sudo apt update && sudo rosdep init && rosdep update
RUN rosdep install --from-paths ./src/task_manager --ignore-src -y -r
RUN sudo rm -rf /workspace/src/
# CV-bridge
RUN apt update
RUN apt install -y ros-humble-nav2-msgs

# CycloneDDS
RUN sudo apt-get update -qq && sudo apt-get install -y -qq ros-humble-rmw-cyclonedds-cpp \
    && sudo rm -rf /var/lib/apt/lists/*
COPY docker/cyclonedds_setup.sh /usr/local/bin/cyclonedds_setup.sh
RUN sudo chmod +x /usr/local/bin/cyclonedds_setup.sh && sudo /usr/local/bin/cyclonedds_setup.sh
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml
RUN echo "source /usr/local/bin/cyclonedds_setup.sh" >> /home/ros/.bashrc
