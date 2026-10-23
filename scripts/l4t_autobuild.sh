#!/bin/bash
# Builds l4t images locally on the Orin when origin/main changes (no push,
# this is the only machine that runs them). Triggered by the systemd timer.
#
# Needs buildx on the default "docker" driver, not "docker-container" —
# the latter can't see local images, so FROM l4t_base pulls from Docker Hub.
#
# Usage: bash scripts/l4t_autobuild.sh [--force]

set -euo pipefail

# Own clone, not the dev's working copy — safe to checkout/switch branches.
REPO_DIR="${L4T_AUTOBUILD_DIR:-$HOME/l4t-autobuild/home2}"
REPO_URL="https://github.com/RoBorregos/home2.git"
STATE_FILE="$REPO_DIR/.l4t_autobuild_last_sha"
LOCK_FILE="/tmp/l4t_autobuild.lock"
LOG_PREFIX="[l4t-autobuild]"

FORCE=""
[ "${1:-}" = "--force" ] && FORCE="true"

exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "$LOG_PREFIX another run is already in progress, skipping."
  exit 0
fi

if [ ! -d "$REPO_DIR/.git" ]; then
  echo "$LOG_PREFIX no clone at $REPO_DIR yet, cloning..."
  mkdir -p "$(dirname "$REPO_DIR")"
  git clone --quiet "$REPO_URL" "$REPO_DIR"
fi

cd "$REPO_DIR"

echo "$LOG_PREFIX fetching origin/main..."
git fetch origin main --quiet

REMOTE_SHA="$(git rev-parse origin/main)"
LAST_SHA="$(cat "$STATE_FILE" 2>/dev/null || true)"

if [ -z "$LAST_SHA" ]; then
  echo "$LOG_PREFIX no previous state, recording $REMOTE_SHA as baseline (no build)."
  echo "$REMOTE_SHA" > "$STATE_FILE"
  exit 0
fi

if [ "$LAST_SHA" = "$REMOTE_SHA" ] && [ -z "$FORCE" ]; then
  echo "$LOG_PREFIX no new commits on main since $LAST_SHA. Nothing to do."
  exit 0
fi

CHANGED="$(git diff --name-only "$LAST_SHA" "$REMOTE_SHA")"
echo "$LOG_PREFIX main moved $LAST_SHA -> $REMOTE_SHA"

base_changed() {
  [ -n "$FORCE" ] && return 0
  echo "$CHANGED" | grep -qE '^docker/(Dockerfile\.ROS-l4t|l4t\.yaml)$'
}

area_changed() {
  local area="$1"
  [ -n "$FORCE" ] && return 0
  echo "$CHANGED" | grep -q "^docker/${area}/"
}

git -c advice.detachedHead=false checkout "$REMOTE_SHA" --quiet

FAILED=()

build_image() {
  # context, tag, dockerfile, then any extra --build-arg ... pairs
  local context="$1" tag="$2" file="$3"; shift 3
  echo "$LOG_PREFIX building $tag ($file, context=$context)"
  if docker buildx build \
      --platform linux/arm64 \
      -f "$file" \
      --tag "$tag" \
      --load \
      "$@" \
      "$context"; then
    echo "$LOG_PREFIX built $tag"
  else
    echo "$LOG_PREFIX FAILED: $tag" >&2
    FAILED+=("$tag")
  fi
}

REBUILD_BASE=false
if base_changed || ! docker image inspect roborregos/home2:l4t_base > /dev/null 2>&1; then
  REBUILD_BASE=true
  build_image "$REPO_DIR/docker" "roborregos/home2:l4t_base" "docker/Dockerfile.ROS-l4t" \
    --build-arg BASE_IMAGE=ubuntu:24.04 \
    --build-arg ROS_DISTRO=jazzy \
    --build-arg USER_UID=1000 \
    --build-arg USER_GID=1000
fi

if area_changed hri || [ "$REBUILD_BASE" = true ]; then
  build_image "$REPO_DIR" "roborregos/home2:hri-l4t" "docker/hri/dockerfiles/Dockerfile.ROS" \
    --build-arg BASE_IMAGE=roborregos/home2:l4t_base
  build_image "$REPO_DIR" "roborregos/home2:hri-stt-l4t" "docker/hri/dockerfiles/Dockerfile.stt-l4t" \
    --build-arg BASE_IMAGE=roborregos/home2:l4t_base
  build_image "$REPO_DIR" "roborregos/home2:hri-tts-l4t" "docker/hri/dockerfiles/Dockerfile.tts-l4t" \
    --build-arg BASE_IMAGE=roborregos/home2:l4t_base
  build_image "$REPO_DIR/docker/hri" "roborregos/home2:hri-ollama-l4t" "docker/hri/dockerfiles/Dockerfile.ollama" \
    --build-arg BASE_IMAGE=roborregos/home2:l4t_base
fi

if area_changed vision || [ "$REBUILD_BASE" = true ]; then
  build_image "$REPO_DIR" "roborregos/home2:vision-l4t" "docker/vision/Dockerfile.l4t" \
    --build-arg BASE_IMAGE=roborregos/home2:l4t_base
fi

if area_changed navigation || [ "$REBUILD_BASE" = true ]; then
  build_image "$REPO_DIR" "roborregos/home2:navigation-l4t" "docker/navigation/Dockerfile.l4t" \
    --build-arg BASE_IMAGE=roborregos/home2:l4t_base
fi

if area_changed manipulation || [ "$REBUILD_BASE" = true ]; then
  build_image "$REPO_DIR" "roborregos/home2:manipulation-l4t" "docker/manipulation/Dockerfile.l4t" \
    --build-arg BASE_IMAGE=roborregos/home2:l4t_base
fi

if area_changed integration || [ "$REBUILD_BASE" = true ]; then
  build_image "$REPO_DIR" "roborregos/home2:integration-l4t" "docker/integration/Dockerfile" \
    --build-arg BASE_IMAGE=roborregos/home2:l4t_base
fi

git checkout main --quiet
git merge --ff-only "$REMOTE_SHA" --quiet
echo "$REMOTE_SHA" > "$STATE_FILE"

if [ "${#FAILED[@]}" -gt 0 ]; then
  echo "$LOG_PREFIX done with failures: ${FAILED[*]}" >&2
  exit 1
fi

echo "$LOG_PREFIX done, main is now at $REMOTE_SHA"
