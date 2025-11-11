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
  elif [ -f "docker/frida_interfaces_cache/docker-compose-${ENV_TYPE}.yml" ]; then
    compose_yaml="docker/frida_interfaces_cache/docker-compose-${ENV_TYPE}.yml"
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
  if [ -z "${INPUT:-}" ]; then
    echo "input not defined" >&2
    return 1
  fi
  if [ ! -d "docker/$INPUT" ]; then
    echo "docker/$INPUT does not exist" >&2
    return 1
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
  local op=${1:-}
  if [ -z "$op" ]; then
    echo "Usage: control --stop|--down" >&2
    return 2
  fi

  local op_flag msg success_msg fail_msg kill_tmux
  case "$op" in
    --stop)
      op_flag="--stop"
      msg="stop"
      kill_tmux=true
      ;;
    --down)
      op_flag="--down"
      msg="down"
      kill_tmux=false
      ;;
    *)
      echo "Unsupported op: $op" >&2
      return 2
      ;;
  esac

  if $kill_tmux && command -v tmux >/dev/null 2>&1 && tmux ls >/dev/null 2>&1; then
    tmux kill-server || true
  fi

  PARALLEL=${PARALLEL:-$(detect_cores)}
  pids=()
  areas_launched=()

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

  rc=0
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