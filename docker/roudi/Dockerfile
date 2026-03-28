ARG BASE_IMAGE=roborregos/home2:l4t_base
FROM ${BASE_IMAGE}

# Build iceoryx from source with expanded publisher/subscriber limits
# (default 512/1024 is too low when navigation + vision + ZED all use SHM)
RUN cd /tmp && \
    git clone --depth 1 -b v2.0.6 https://github.com/eclipse-iceoryx/iceoryx.git && \
    cd iceoryx && \
    cmake -S iceoryx_meta -B build \
        -DCMAKE_INSTALL_PREFIX=/opt/ros/humble \
        -DIOX_MAX_PUBLISHERS=2048 \
        -DIOX_MAX_SUBSCRIBERS=2048 \
        -DBUILD_BINDING_C=ON \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build -j$(nproc) && \
    cmake --install build && \
    cd / && rm -rf /tmp/iceoryx

COPY roudi_config.toml /etc/iceoryx/roudi_config.toml

CMD ["/opt/ros/humble/bin/iox-roudi", "-c", "/etc/iceoryx/roudi_config.toml"]
