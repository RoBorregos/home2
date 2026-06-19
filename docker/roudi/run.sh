#!/bin/bash
source ../../lib.sh

ACTION=${1:-up}

# Detect environment
if [ -f /etc/nv_tegra_release ]; then
  ENV_TYPE=l4t
elif command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
  ENV_TYPE=cuda
else
  ENV_TYPE=cpu
fi

# Jetson Thor uses the l4t_thor base + its own roudi Dockerfile (Noble/GCC 13).
# Opt in with PLATFORM=thor (repo-root .env, sourced by lib.sh) or auto-detect.
ROUDI_DOCKERFILE="docker/roudi/Dockerfile"
if [ "${PLATFORM:-}" = "thor" ] || \
   { [ "$ENV_TYPE" = "l4t" ] && tr -d '\0' < /proc/device-tree/model 2>/dev/null | grep -qi thor; }; then
  ENV_TYPE=l4t_thor
  ROUDI_DOCKERFILE="docker/roudi/Dockerfile.l4t_thor"
fi

echo "" > .env
add_or_update_variable .env "BASE_IMAGE" "roborregos/home2:${ENV_TYPE}_base"
add_or_update_variable .env "IMAGE_NAME" "roborregos/home2:roudi-${ENV_TYPE}"
add_or_update_variable .env "ROUDI_DOCKERFILE" "$ROUDI_DOCKERFILE"

case $ACTION in
  --down)
    docker compose down
    exit 0
    ;;
  --stop)
    docker compose stop
    exit 0
    ;;
  *)
    # Build image if needed
    check_image_exists "roborregos/home2:roudi-${ENV_TYPE}"
    if [ $? -eq 1 ]; then
      docker compose build
    fi
    docker compose up -d
    ;;
esac
