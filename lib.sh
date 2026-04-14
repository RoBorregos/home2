#!/usr/bin/env bash

AREAS="vision manipulation navigation integration hri zed"
ORIN_SERVER_AREAS="hri"

# --- guard against multiple sourcing ---
if [[ -n "${__HOME2_LIB_SOURCED:-}" ]]; then
  return 0
fi
__HOME2_LIB_SOURCED=1

# --- load .env ---
if [ -f ".env" ]; then
  source .env
fi

# --- helpers ---

# Run a command on the remote Orin server via SSH.
orin_ssh() {
  sshpass -p "${ORIN_SSH_PASS}" ssh -o StrictHostKeyChecking=no "${ORIN_SSH_USER}@${ORIN_SSH_HOST}" "$@"
}

# Check if an area should run on the remote Orin server.
is_orin_area() {
  echo " ${ORIN_SERVER_AREAS} " | grep -q " $1 "
}

check_image_exists() {
  local image_name=$1
  if ! docker images --format "{{.Repository}}:{{.Tag}}" | grep -q "^${image_name}$"; then
    echo "Image $image_name does not exist. Building it..."
    return 1
  else
    echo "Image $image_name already exists. Skipping build."
    return 0
  fi
}

# Upload a docker image to DockerHub.
upload_image() {
  local image="$1"
  echo "Pushing image: $image"
  if docker push "$image"; then
    echo "Successfully pushed: $image"
  else
    echo "Error: Failed to push $image" >&2
    return 1
  fi
}

# Ensure logged in to DockerHub as roborregos.
docker_login() {
  local current_user
  current_user=$(docker system info 2>/dev/null | grep "Username:" | awk '{print $2}')

  if [ "$current_user" = "roborregos" ]; then
    echo "Already logged in as roborregos."
    return 0
  fi

  if [ -n "$current_user" ]; then
    echo "Error: logged in as '$current_user', must be 'roborregos'. Run: docker logout && docker login" >&2
    return 1
  fi

  echo "Not logged in to DockerHub. Please log in as roborregos:"
  docker login || return 1
}

# Ensure an image exists locally (build if missing) then push it.
ensure_and_upload_image() {
  local image="$1"
  local compose_file="$2"
  shift 2
  local extra_args=("$@")

  docker_login || return 1

  if docker image inspect "$image" > /dev/null 2>&1; then
    echo "Image $image found locally, skipping build."
  else
    if [ -z "$compose_file" ]; then
      echo "Error: image $image not found locally and no compose file provided to build it." >&2
      return 1
    fi
    echo "Image $image not found locally. Building with: docker compose -f $compose_file build ${extra_args[*]}"
    docker compose -f "$compose_file" build "${extra_args[@]}" || { echo "Build failed for $image" >&2; return 1; }
  fi

  upload_image "$image"
}

add_or_update_variable() {
  local file=$1 variable=$2 value=$3

  # escape sed-sensitive chars in value
  local escaped_value
  escaped_value=$(printf '%s\n' "$value" | sed -e 's/[&/\]/\\&/g')

  if grep -q "^${variable}=" "$file" 2>/dev/null; then
    sed -i "s|^${variable}=.*|${variable}=${escaped_value}|" "$file"
  else
    printf '%s=%s\n' "$variable" "$value" >> "$file"
  fi
}

# Parse flags shared by all area run.sh scripts.
# First arg: compose file path (e.g. "docker-compose.yaml"); remaining args are the script's $@.
parse_common_flags() {
  local compose_file="${1:-}"
  shift
  local compose="${compose_file:+docker compose -f $compose_file}"
  compose="${compose:-docker compose}"

  DETACHED=""
  BUILD=""
  BUILD_IMAGE=""
  UPLOAD_IMAGE=""

  for arg in "$@"; do
    case $arg in
      "-d")             DETACHED="-d" ;;
      "--build")        BUILD="true" ;;
      "--build-image")  BUILD_IMAGE="--build" ;;
      "--upload-image") UPLOAD_IMAGE="true" ;;
      "--clean")        clean_directories ;;
      "--recreate")     $compose down ;;
      "--down")         $compose down; exit 0 ;;
      "--stop")         $compose stop; exit 0 ;;
    esac
  done
}

# Write .env variables shared by all areas.
# First arg: area name. Second arg: env file path (default: .env).
setup_common_env() {
  local area="$1"
  local env_file="${2:-.env}"

  echo "" > "$env_file"

  if [ -f /etc/cyclonedds.env ]; then
    source /etc/cyclonedds.env
  fi

  add_or_update_variable "$env_file" "CYCLONE_INTERFACE" "${CYCLONE_INTERFACE:-}"
  add_or_update_variable "$env_file" "LOCAL_USER_ID"     "$(id -u)"
  add_or_update_variable "$env_file" "LOCAL_GROUP_ID"    "$(id -g)"
  add_or_update_variable "$env_file" "BASE_IMAGE"        "roborregos/home2:${ENV_TYPE}_base"
  add_or_update_variable "$env_file" "IMAGE_NAME"        "roborregos/home2:${area}-${ENV_TYPE}"
  
  mkdir -p install build log
}

clean_directories() {
  local target="${1:-.}"
  echo "Cleaning build/log/install in: $target"
  rm -rf "$target/build" "$target/log" "$target/install"
}

run_frida_interfaces() {
  local compose_yaml
  if [ -f "docker/frida_interfaces_cache/docker-compose-${ENV_TYPE}.yaml" ]; then
    compose_yaml="docker/frida_interfaces_cache/docker-compose-${ENV_TYPE}.yaml"
  else
    echo "frida_interfaces cache compose file not found for env=${ENV_TYPE}" >&2
    return 1
  fi

  echo "Running frida_interfaces_cache to build frida_interfaces (using $compose_yaml)"
  mkdir -p "docker/frida_interfaces_cache/build" "docker/frida_interfaces_cache/install" "docker/frida_interfaces_cache/log"
  export GID=$(id -g) && docker compose -f "$compose_yaml" run --rm frida_interfaces_cache
}

run_area() {
  if [ "$INPUT" != "zed" ] && [ ! -d "docker/frida_interfaces_cache/build" ]; then
    echo "Cache directory missing. Building frida_interfaces_cache first..."
    run_frida_interfaces || { echo "frida_interfaces cache build failed" >&2; return 1; }
  fi

  # Auto-detect Jetson for SHM default
  if [ -z "${CYCLONE_SHM:-}" ]; then
    if [ -f /etc/nv_tegra_release ]; then
      export CYCLONE_SHM=1
    else
      export CYCLONE_SHM=0
    fi
  fi

  # Start RouDi container for SHM-enabled areas (zed, vision, navigation)
  if [ "${CYCLONE_SHM}" = "1" ]; then
    if [ "$INPUT" = "zed" ] || [ "$INPUT" = "vision" ] || [ "$INPUT" = "navigation" ]; then
      ensure_roudi || { echo "RouDi startup failed" >&2; return 1; }
    fi
  fi

  echo "Running image from $INPUT"
  (cd "docker/$INPUT" && bash "./run.sh" "${@:2}" "${ENV_TYPE}")
  return $?
}

detect_cores() {
  if command -v nproc >/dev/null 2>&1; then
    nproc
  else
    sysctl -n hw.ncpu 2>/dev/null || echo 4
  fi
}

control() {
  # No input validation required; accept whatever flag is passed and forward it.
  local op_flag=${1:-}
  local msg="${op_flag#--}"

  # TODO: instead of killing the whole tmux server, only kill home-related sessions.
  if command -v tmux >/dev/null 2>&1 && tmux ls >/dev/null 2>&1; then
    tmux kill-server || true
  fi
  orin_ssh "tmux kill-server" 2>/dev/null || true

  PARALLEL=${PARALLEL:-$(detect_cores)}
  local pids=()
  local areas_launched=()

  # Launch each area's run.sh in parallel up to $PARALLEL children. Each area is started in its directory and passed the control flag + environment type.
  for area in ${AREAS:-}; do
    AREA_RUN="docker/${area}/run.sh"
    if [ -f "$AREA_RUN" ]; then
      while [ "$(jobs -rp | wc -l)" -ge "$PARALLEL" ]; do
        sleep 0.1
      done

      if is_orin_area "${area}"; then
        orin_ssh "cd $(pwd)/docker/${area} && bash ./run.sh ${op_flag} ${ENV_TYPE}" &
      else
        ( cd "docker/${area}" && bash "./run.sh" "${op_flag}" "${ENV_TYPE}" ) &
      fi
      pids+=($!)
      areas_launched+=("$area")
    fi
  done

  # Wait for launched area processes and report per-area result
  local rc=0
  for i in "${!pids[@]}"; do
    pid=${pids[i]}
    area=${areas_launched[i]}
    if wait "$pid"; then
      echo "Area ${area}: ${msg} completed"
    else
      echo "Area ${area}: ${msg} failed" >&2
      rc=1
    fi
  done

  # Stop RouDi container on --down
  if [ "$op_flag" = "--down" ]; then
    if docker ps -a --format '{{.Names}}' | grep -q '^home2-roudi$'; then
      echo "Stopping RouDi container..."
      (cd "docker/roudi" && docker compose down)
    fi
  fi

  # Clean stale iceoryx artifacts when SHM is disabled
  if [ "${CYCLONE_SHM:-0}" != "1" ] && [ "$op_flag" = "--down" ]; then
    rm -f /tmp/iox-unique-roudi.lock /tmp/roudi.lock 2>/dev/null
    rm -f /dev/shm/iceoryx* 2>/dev/null
  fi

  echo "All ${msg}s attempted."
  return $rc
}

run_task() {
  for area in ${AREAS}; do
    SESSION_NAME=$area

    if is_orin_area "${area}"; then
      orin_ssh "tmux new-session -d -s '${SESSION_NAME}' && tmux send-keys -t '${SESSION_NAME}' 'cd $(pwd) && bash run.sh $area $*' C-m"
    else
      tmux new-session -d -s "$SESSION_NAME"
      tmux send-keys -t "$SESSION_NAME" "bash run.sh $area $*" C-m
    fi
  done
}

ensure_roudi() {
  local roudi_container="home2-roudi"
  if docker ps --format '{{.Names}}' | grep -q "^${roudi_container}$"; then
    echo "[RouDi] Already running."
    return 0
  fi
  echo "[RouDi] Starting dedicated RouDi container..."
  (cd "docker/roudi" && bash ./run.sh)
  # Wait for RouDi to be healthy
  local retries=10
  while [ $retries -gt 0 ]; do
    if docker ps --format '{{.Names}}' | grep -q "^${roudi_container}$"; then
      echo "[RouDi] Container is up."
      sleep 1
      return 0
    fi
    sleep 1
    retries=$((retries - 1))
  done
  echo "[RouDi] WARNING: RouDi container did not start in time." >&2
  return 1
}

update_map(){
  local map_flag=${2:-}
  echo "Updating actual map to $map_flag"

  local constant_source_file="$HOME/.bashrc"

  if [ -f "$HOME/.zshrc" ]; then
    echo "ZSHELL DETECTED"
    constant_source_file="$HOME/.zshrc"
  fi

  if grep -q "export MAP_NAME=" "$constant_source_file"; then
      sed -i "s|export MAP_NAME=.*|export MAP_NAME=\"$map_flag\"|" "$constant_source_file"
  else
      echo "export MAP_NAME=\"$map_flag\"" >> "$constant_source_file"
  fi

  echo "REMEMBER TO SOURCE $constant_source_file TO BE ABLE TO USE MAP" 
}
