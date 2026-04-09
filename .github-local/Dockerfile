# ros2-build equivalent as dockerfile for local debugging
FROM ros:humble AS ros2-build

WORKDIR /ros2_ws

# CycloneDDS
RUN sudo apt-get update -qq && sudo apt-get install -y -qq ros-humble-rmw-cyclonedds-cpp \
    && sudo rm -rf /var/lib/apt/lists/*
COPY docker/cyclonedds_setup.sh /usr/local/bin/cyclonedds_setup.sh
RUN sudo chmod +x /usr/local/bin/cyclonedds_setup.sh && sudo /usr/local/bin/cyclonedds_setup.sh
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml
RUN echo "source /usr/local/bin/cyclonedds_setup.sh" >> /home/ros/.bashrc
