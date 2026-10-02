#!/bin/bash
# Builds OpenCV + contrib with CUDA/cuDNN into /usr/local and removes the build tree.
# Replaces the downloaded AastaNV/JEP script that used to be sed-patched per area.
set -euo pipefail

OPENCV_VERSION="${OPENCV_VERSION:-4.14.0}"
CUDA_ARCH_BIN="${CUDA_ARCH_BIN:-8.7}"
JOBS="${JOBS:-6}"
PY_SITE="/usr/local/lib/python3.12/dist-packages"

apt-get update
apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config curl \
    libgtk2.0-dev libavcodec-dev libavformat-dev libswscale-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libtbb12 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libv4l-dev v4l-utils \
    python3-dev
rm -rf /var/lib/apt/lists/*

WORKDIR="$(mktemp -d)"
cd "$WORKDIR"
curl -sSL "https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.tar.gz" | tar xz
curl -sSL "https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.tar.gz" | tar xz

cmake -S "opencv-${OPENCV_VERSION}" -B build \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D CMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -D OPENCV_EXTRA_MODULES_PATH="${WORKDIR}/opencv_contrib-${OPENCV_VERSION}/modules" \
    -D WITH_CUDA=ON -D WITH_CUDNN=ON -D OPENCV_DNN_CUDA=ON \
    -D CUDA_ARCH_BIN="${CUDA_ARCH_BIN}" -D CUDA_ARCH_PTX="" \
    -D CUDA_NVCC_FLAGS="-std=c++17" \
    -D BUILD_opencv_cudacodec=OFF \
    -D WITH_GSTREAMER=ON -D WITH_LIBV4L=ON \
    -D BUILD_opencv_python3=ON \
    -D PYTHON3_EXECUTABLE=/usr/bin/python3 \
    -D OPENCV_PYTHON3_INSTALL_PATH="${PY_SITE}" \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF -D BUILD_DOCS=OFF
cmake --build build -j"${JOBS}"
cmake --install build
ldconfig

cd /
rm -rf "$WORKDIR"
