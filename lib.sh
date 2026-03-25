#!/usr/bin/env bash

AREAS="vision manipulation navigation integration hri"

# --- guard against multiple sourcing ---
if [[ -n "${__HOME2_LIB_SOURCED:-}" ]]; then
  return 0
fi
__HOME2_LIB_SOURCED=1

# --- helpers ---

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
  CLEAN=""

  for arg in "$@"; do
    case $arg in
      "-d")             DETACHED="-d" ;;
      "--build")        BUILD="true" ;;
      "--build-image")  BUILD_IMAGE="--build" ;;
      "--upload-image") UPLOAD_IMAGE="true" ;;
      "--clean")        CLEAN="true" ;;
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

  clean_workspace_directories
  mkdir -p install build log
}

clean_workspace_directories() {
  if [ "$CLEAN" == "true" ]; then
    echo "Cleaning build/log/install directories..."
    rm -rf build log install
  fi
}

clean_frida_interfaces() {
  echo "Cleaning frida_interfaces_cache build/log/install..."
  rm -rf "docker/frida_interfaces_cache/build" \
         "docker/frida_interfaces_cache/log" \
         "docker/frida_interfaces_cache/install"
  echo "frida_interfaces_cache cleaned."
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
  if [ ! -d "docker/frida_interfaces_cache/build" ]; then
    echo "Cache directory missing. Building frida_interfaces_cache first..."
    run_frida_interfaces || { echo "frida_interfaces cache build failed" >&2; return 1; }
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

  PARALLEL=${PARALLEL:-$(detect_cores)}
  local pids=()
  local areas_launched=()

  # Launch each area's run.sh in parallel up to $PARALLEL children.
  # Each area is started in its directory and passed the control flag + environment type.
  for area in ${AREAS:-}; do
    AREA_RUN="docker/${area}/run.sh"
    if [ -f "$AREA_RUN" ]; then
      while [ "$(jobs -rp | wc -l)" -ge "$PARALLEL" ]; do
        sleep 0.1
      done
      ( cd "docker/${area}" && bash "./run.sh" "${op_flag}" "${ENV_TYPE}" ) &
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

  echo "All ${msg}s attempted."
  return $rc
}

run_task() {
  local task=$1
  for area in ${AREAS}; do
    SESSION_NAME=$area
    tmux new-session -d -s "$SESSION_NAME"
    tmux send-keys -t "$SESSION_NAME" "bash run.sh $area $task" C-m
  done
}