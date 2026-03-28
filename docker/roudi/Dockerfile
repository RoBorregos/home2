ARG BASE_IMAGE=roborregos/home2:l4t_base
FROM ${BASE_IMAGE}

# Build iceoryx from source with expanded publisher/subscriber limits
# (default 512/1024 is too low when navigation + vision + ZED all use SHM)
RUN apt-get update -qq && apt-get install -y --no-install-recommends libacl1-dev && rm -rf /var/lib/apt/lists/* && \
    dpkg --force-depends -r ros-humble-iceoryx-posh ros-humble-iceoryx-hoofs ros-humble-iceoryx-binding-c 2>/dev/null || true
RUN cd /tmp && \
    git clone --depth 1 -b v2.0.6 https://github.com/eclipse-iceoryx/iceoryx.git && \
    cd iceoryx && \
    cmake -S iceoryx_meta -B build \
        -DCMAKE_INSTALL_PREFIX=/opt/ros/humble \
        -DIOX_MAX_PUBLISHERS=20000 \
        -DIOX_MAX_SUBSCRIBERS=20000 \
        -DBUILD_BINDING_C=ON \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build -j$(nproc) && \
    cmake --install build && \
    ldconfig && \
    cd / && rm -rf /tmp/iceoryx

COPY roudi_config.toml /etc/iceoryx/roudi_config.toml

CMD ["/opt/ros/humble/bin/iox-roudi", "-c", "/etc/iceoryx/roudi_config.toml"]
