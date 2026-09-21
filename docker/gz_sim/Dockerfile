ARG BASE_IMAGE=ros:jazzy
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=jazzy
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV LANG=C.UTF-8

# Non-root user matching the host
RUN (id -u ubuntu >/dev/null 2>&1 && userdel -r ubuntu || true) \
    && (getent group ubuntu >/dev/null 2>&1 && groupdel ubuntu || true) \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Gazebo Harmonic, ros2_control, MoveIt and perception dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-ros-perception \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-laser-geometry \
    ros-${ROS_DISTRO}-sensor-msgs-py \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-generate-parameter-library \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-dev-tools \
    libpcl-dev libeigen3-dev libopencv-dev \
    python3-pip python3-scipy python3-transforms3d python3-matplotlib \
    git wget curl mesa-utils libegl1 libgl1 \
    && rm -rf /var/lib/apt/lists/*

# Remaining system deps resolved from the package manifests
COPY frida_interfaces /tmp/src/frida_interfaces
COPY frida_constants /tmp/src/frida_constants
COPY robot_description /tmp/src/robot_description
COPY manipulation/packages /tmp/src/manipulation
COPY vision/packages/object_detector_2d/package.xml /tmp/src/object_detector_2d/package.xml
COPY task_manager/package.xml /tmp/src/task_manager/package.xml
RUN apt-get update && (rosdep init || true) && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /tmp/src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
       --skip-keys "gpd opencv message_package eigen3 pcl-ros gazebo_ros gazebo_plugins gazebo_ros2_control libpcl-dev libvtk-qt mujoco_ros2_control" \
    ; rm -rf /tmp/src /var/lib/apt/lists/*

# Detector runtime (same versions as the vision image)
RUN pip3 install --no-cache-dir torch==2.6.0 torchvision==0.21.0 --index-url https://download.pytorch.org/whl/cu124 \
    && pip3 install --no-cache-dir ultralytics==8.4.19 imutils lap "numpy<2"

# CycloneDDS, network only
COPY docker/cyclonedds_setup.sh /usr/local/bin/cyclonedds_setup.sh
RUN chmod +x /usr/local/bin/cyclonedds_setup.sh && /usr/local/bin/cyclonedds_setup.sh
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# GPD grasp detector library (arm_pkg links against it)
COPY manipulation/packages/gpd /tmp/gpd
RUN cd /tmp/gpd && rm -rf build && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j"$(nproc)" && make install \
    && ldconfig && rm -rf /tmp/gpd

RUN mkdir -p /workspace && chown -R $USERNAME:$USERNAME /workspace \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

USER $USERNAME
WORKDIR /workspace
