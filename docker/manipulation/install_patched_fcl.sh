#!/bin/bash
# Rebuilds apt's FCL 0.7.0 so a present but empty octomap no longer tests every robot triangle.
set -e

PATCH="$(dirname "$(readlink -f "$0")")/fcl_empty_octree.patch"
SRC=$(mktemp -d)

apt-get update
apt-get install -y cmake curl g++ libccd-dev libeigen3-dev liboctomap-dev patch

curl -fsSL https://github.com/flexible-collision-library/fcl/archive/refs/tags/0.7.0.tar.gz \
    | tar xz -C "$SRC" --strip-components=1
patch -d "$SRC" -p1 < "$PATCH"

cmake -S "$SRC" -B "$SRC/build" -DCMAKE_BUILD_TYPE=Release -DFCL_WITH_OCTOMAP=ON \
    -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=/usr \
    -DCMAKE_INSTALL_LIBDIR="lib/$(gcc -dumpmachine)"
cmake --build "$SRC/build" -j"$(nproc)"
cmake --install "$SRC/build"

# Keep apt from restoring the unpatched library
apt-mark hold libfcl0.7 libfcl-dev
rm -rf "$SRC" /var/lib/apt/lists/*
