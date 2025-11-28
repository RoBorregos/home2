#!/usr/bin/env bash

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

run_frida_interfaces() {
  local compose_yaml
  if [ -f "docker/frida_interfaces_cache/docker-compose-${ENV_TYPE}.yaml" ]; then
    compose_yaml="docker/frida_interfaces_cache/docker-compose-${ENV_TYPE}.yaml"
  else
    echo "frida_interfaces cache compose file not found for env=${ENV_TYPE}" >&2
    return 1
  fi

  echo "Running frida_interfaces_cache to build frida_interfaces (using $compose_yaml)"
  docker compose -f "$compose_yaml" run --rm frida_interfaces_cache
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

  # Launch each area's run.sh in parallel up to $PARALLEL children. Each area is started in its directory and passed the control flag + environment type.
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

# Build and push all roborregos/home2 images referenced in a compose file
upload_images() {
  local compose_file=${1:-docker-compose.yaml}
  if [ ! -f "$compose_file" ]; then
    echo "Compose file $compose_file not found" >&2
    return 1
  fi

  docker login

  echo "Building images from compose: $compose_file"
  if ! docker compose -f "$compose_file" build; then
    echo "Compose build failed" >&2
    return 1
  fi

  local images
  images=$(docker compose -f "$compose_file" config 2>/dev/null \
    | awk '/^\s*image:/ {print $2}' | sort -u || true)
  images=$(echo "$images" | grep '^roborregos/home2' || true)

  if [ -z "$images" ]; then
    echo "Nothing to push."
    return 1
  fi

  local rc=0
  echo "Images to push:"
  echo "$images"
  while IFS= read -r img; do
    [ -z "$img" ] && continue
    echo "Pushing $img ..."
    if ! docker push "$img"; then
      echo "Failed to push $img" >&2
      rc=1
    fi
  done <<< "$images"

  if [ $rc -eq 0 ]; then
    echo "All pushes finished."
  else
    echo "One or more pushes failed." >&2
  fi
  return $rc

}

# Build and pull all roborregos/home2 images referenced in the env/compose file
pull_image() {
  if [ "$1" = "--env-file" ]; then
    local envfile=${2:-.env}
    if [ ! -f "$envfile" ]; then
      echo "Env file $envfile not found" >&2
      return 1
    fi
    local image
    image=$(grep -E '^IMAGE_NAME=' "$envfile" | cut -d'=' -f2- | tr -d '"' | tr -d "'")
    if [ -z "$image" ]; then
      echo "IMAGE_NAME not found in $envfile" >&2
      return 1
    fi
    echo "Pulling image from env: $image"
    if ! docker pull "$image"; then
      echo "Failed to pull $image" >&2
      return 1
    fi
    return 0

  elif [ "$1" = "--compose" ]; then
    local compose_file=${2:-docker-compose.yaml}
    if [ ! -f "$compose_file" ]; then
      echo "Compose file $compose_file not found" >&2
      return 1
    fi

    local images
    images=$(docker compose -f "$compose_file" config 2>/dev/null \
      | awk '/^\s*image:/ {print $2}' | sort -u || true)
    images=$(echo "$images" | grep '^roborregos/home2' || true)

    if [ -z "$images" ]; then
      echo "No matching images to pull from $compose_file."
      return 1
    fi

    local rc=0
    echo "Images to pull from $compose_file:"
    echo "$images"
    while IFS= read -r img; do
      [ -z "$img" ] && continue
      echo "Pulling $img ..."
      if ! docker pull "$img"; then
        echo "Failed to pull $img" >&2
        rc=1
      fi
    done <<< "$images"

    if [ $rc -ne 0 ]; then
      echo "One or more pulls failed." >&2
    else
      echo "All pulls finished."
    fi
    return $rc

  else
    local image=${1:-}
    if [ -z "$image" ]; then
      echo "pull_image: no image specified" >&2
      return 1
    fi
    echo "Pulling image: $image"
    if ! docker pull "$image"; then
      echo "Failed to pull $image" >&2
      return 1
    fi
    return 0
  fi
}