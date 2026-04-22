ARG BASE_IMAGE
FROM ${BASE_IMAGE}

# rosbridge + web_video_server + nodejs (for Next.js)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbridge-server \
    ros-humble-web-video-server \
    libacl1-dev \
    && curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/*

USER ros
RUN sudo chown -R ros /usr/lib/python3.10/ /usr/local/lib/python3.10/ 2>/dev/null || true

# CycloneDDS with SHM (iceoryx zero-copy) — needed for ZED → display frame path
RUN sudo dpkg --force-depends -r ros-humble-iceoryx-posh ros-humble-iceoryx-hoofs ros-humble-iceoryx-binding-c 2>/dev/null || true
RUN cd /tmp && \
    git clone --depth 1 -b v2.0.6 https://github.com/eclipse-iceoryx/iceoryx.git && \
    cd iceoryx && \
    sed -i 's|set(IOX_INTERNAL_MAX_NUMBER_OF_NOTIFIERS 256)|set(IOX_INTERNAL_MAX_NUMBER_OF_NOTIFIERS 1024)|' iceoryx_posh/cmake/IceoryxPoshDeployment.cmake && \
    cmake -S iceoryx_meta -B build \
    -DCMAKE_INSTALL_PREFIX=/opt/ros/humble \
    -DIOX_MAX_PUBLISHERS=20000 \
    -DIOX_MAX_SUBSCRIBERS=20000 \
    -DIOX_MAX_CHUNKS_HELD_PER_SUBSCRIBER_SIMULTANEOUSLY=2048 \
    -DBUILD_BINDING_C=ON \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build -j$(nproc) && \
    sudo cmake --install build && \
    sudo ldconfig && \
    cd / && rm -rf /tmp/iceoryx

RUN cd /tmp && \
    git clone --depth 1 -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git && \
    cd cyclonedds && mkdir build && cd build && \
    . /opt/ros/humble/setup.sh && \
    cmake .. \
    -DCMAKE_INSTALL_PREFIX=/opt/ros/humble \
    -DCMAKE_PREFIX_PATH=/opt/ros/humble \
    -DENABLE_SHM=ON \
    -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && sudo make install && \
    cd / && rm -rf /tmp/cyclonedds

COPY docker/cyclonedds_setup.sh /usr/local/bin/cyclonedds_setup.sh
COPY roudi_config.toml /etc/iceoryx/roudi_config.toml
ENV CYCLONE_SHM=1
RUN sudo chmod +x /usr/local/bin/cyclonedds_setup.sh && sudo /usr/local/bin/cyclonedds_setup.sh
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml
RUN echo "source /usr/local/bin/cyclonedds_setup.sh" >> /home/ros/.bashrc
