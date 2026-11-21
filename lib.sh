#!/usr/bin/env bash

AREAS="vision manipulation navigation integration hri zed display"
ORIN_SERVER_AREAS="hri"

# --- guard against multiple sourcing ---
if [[ -n "${__HOME2_LIB_SOURCED:-}" ]]; then
  return 0
fi
__HOME2_LIB_SOURCED=1

# --- load repo-root .env (for ORIN SSH creds, etc.) ---
# Anchor to lib.sh's own directory so re-sourcing from a subdirectory does
# not accidentally pull a stale per-area .env into the current shell.
__HOME2_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$__HOME2_LIB_DIR/.env" ]; then
  source "$__HOME2_LIB_DIR/.env"
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
    set_variable_in_file "$file" "$variable" "$escaped_value"
  else
    printf '%s=%s\n' "$variable" "$value" >> "$file"
  fi
}

set_variable_in_file() {
  local file=$1 variable=$2 escaped_value=$3
  sed -i "s|^${variable}=.*|${variable}=${escaped_value}|" "$file"
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

# Print the CycloneDDS interface: env over /etc/cyclonedds.env, lo if it has no IPv4 address.
resolve_cyclone_interface() {
  local iface="${CYCLONE_INTERFACE-}"
  if [ -z "${CYCLONE_INTERFACE+x}" ] && [ -f /etc/cyclonedds.env ]; then
    iface="$(source /etc/cyclonedds.env; echo "${CYCLONE_INTERFACE-}")"
  fi
  if [ -n "$iface" ] && [ "$iface" != "lo" ] && command -v ip >/dev/null \
     && [ -z "$(ip -4 -o addr show dev "$iface" 2>/dev/null)" ]; then
    echo "[CycloneDDS] WARNING: $iface has no IPv4 address (router disconnected?). Using lo: only this machine will see ROS topics." >&2
    iface="lo"
  fi
  echo "$iface"
}

# Write .env variables shared by all areas.
# First arg: area name. Second arg: env file path (default: .env).
setup_common_env() {
  local area="$1"
  local env_file="${2:-.env}"

  echo "" > "$env_file"

  add_or_update_variable "$env_file" "CYCLONE_INTERFACE" "$(resolve_cyclone_interface)"
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
  # simulation builds frida_interfaces itself inside its own workspace
  if [ "$INPUT" != "zed" ] && [ "$INPUT" != "simulation" ] && [ ! -d "docker/frida_interfaces_cache/build" ]; then
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

  # Start RouDi container for SHM-enabled areas (zed, vision, navigation, display, simulation)
  if [ "${CYCLONE_SHM}" = "1" ]; then
    if [ "$INPUT" = "zed" ] || [ "$INPUT" = "vision" ] || [ "$INPUT" = "navigation" ] || [ "$INPUT" = "display" ] || [ "$INPUT" = "simulation" ]; then
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

  # --- Cleanup screen sessions ---
  if command -v screen >/dev/null 2>&1; then
    # Kill local screen sessions named 'frida' or matching areas
    screen -ls | grep -E "\.frida|\.$(echo $AREAS | sed 's/ /|/g')" | cut -d. -f1 | awk '{print $1}' | xargs -I{} screen -X -S {} quit || true
  fi
  orin_ssh "pkill screen" 2>/dev/null || true

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
  SESSION_NAME="frida"

  # Kill existing sessions to start fresh
  screen -S "$SESSION_NAME" -X quit 2>/dev/null || true
  orin_ssh "screen -S $SESSION_NAME -X quit" 2>/dev/null || true

  local local_first=true
  local remote_first=true

  for area in ${AREAS}; do
    # The safety routine intentionally does not bring up vision.
    if [ "$1" = "--safety" ] && [ "$area" = "vision" ]; then
      continue
    fi
    CMD="bash run.sh $area $*"
    if is_orin_area "${area}"; then
      if [ "$remote_first" = true ]; then
        orin_ssh "screen -dmS ${SESSION_NAME} -t ${area} bash -c 'cd $(pwd) && ${CMD}; exec bash'"
        remote_first=false
      else
        orin_ssh "screen -S ${SESSION_NAME} -X screen -t ${area} bash -c 'cd $(pwd) && ${CMD}; exec bash'"
      fi
    else
      if [ "$local_first" = true ]; then
        screen -dmS "$SESSION_NAME" -t "$area" bash -c "$CMD; exec bash"
        local_first=false
      else
        screen -S "$SESSION_NAME" -X screen -t "$area" bash -c "$CMD; exec bash"
      fi
    fi
  done

  echo "Tasks started in screen session '$SESSION_NAME'."
  echo "To view, run: screen -r $SESSION_NAME"
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

# Find the X display of the robot's physical monitor and an Xauthority file
# that grants access to it. Sets HOME2_X_DISPLAY, HOME2_X_AUTH and
# HOME2_X_SESSION_BUS (empty when not known). Returns 1 if none was found.
#
# Order:
#   0. HOME2_DISPLAY env var (manual override, e.g. HOME2_DISPLAY=:10.0)
#   0b. The xrdp (Remmina/RDP) session the command was launched from
#   1. Our own GNOME session on the monitor (skips xrdp sessions)
#   2. GDM login screen's X server, when nobody is logged in on the monitor
#   3. The shell's DISPLAY, if that X server actually exists (xrdp / ssh -X)
# The shell's DISPLAY is deliberately not trusted first: ~/.bashrc on the Orin
# exports a fixed DISPLAY=:1 that does not exist until someone logs in.
resolve_physical_display() {
  HOME2_X_DISPLAY=""
  HOME2_X_AUTH=""
  HOME2_X_SESSION_BUS=""
  local uid gid pid environ disp auth
  uid=$(id -u)
  gid=$(id -g)

  if [ -n "${HOME2_DISPLAY:-}" ]; then
    HOME2_X_DISPLAY="$HOME2_DISPLAY"
    HOME2_X_AUTH="${XAUTHORITY:-$HOME/.Xauthority}"
    return 0
  fi

  # 0b. Launched from a terminal inside an xrdp (Remmina/RDP) session: show
  #     GUIs there. DISPLAY can't be trusted (~/.bashrc overrides it), but
  #     xrdp's own variables carry the display number (..._socket_<N>).
  if [ -n "${XRDP_SESSION:-}" ]; then
    local xrdp_num="${XRDP_PULSE_SINK_SOCKET##*_}"
    case "$xrdp_num" in ''|*[!0-9]*)
      xrdp_num=$(ps -u "$uid" -o args= | sed -n 's|^[^ ]*Xorg :\([0-9][0-9]*\) .*xrdp.*|\1|p' | head -n 1) ;;
    esac
    auth="$HOME/.Xauthority"
    if [ -n "$xrdp_num" ] && DISPLAY=":$xrdp_num" XAUTHORITY="$auth" xset q >/dev/null 2>&1; then
      HOME2_X_DISPLAY=":$xrdp_num"
      HOME2_X_AUTH="$auth"
      return 0
    fi
  fi

  # 1. Desktop session on the monitor. GDM keeps its Xauthority under
  #    /run/user/<uid>/gdm/; xrdp sessions use ~/.Xauthority instead.
  for pid in $(pgrep -u "$uid" -x gnome-shell 2>/dev/null); do
    environ=$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null)
    disp=$(sed -n 's/^DISPLAY=//p' <<< "$environ" | head -n 1)
    auth=$(sed -n 's/^XAUTHORITY=//p' <<< "$environ" | head -n 1)
    case "$auth" in "/run/user/$uid/gdm/"*) ;; *) continue ;; esac
    if [ -n "$disp" ] && [ -r "$auth" ]; then
      HOME2_X_DISPLAY="$disp"
      HOME2_X_AUTH="$auth"
      HOME2_X_SESSION_BUS=$(sed -n 's/^DBUS_SESSION_BUS_ADDRESS=//p' <<< "$environ" | head -n 1)
      return 0
    fi
  done

  # 2. Login screen: GDM's greeter Xorg runs on a VT (xrdp's Xorg does not).
  #    Its cookie is root-only, so copy it to a file we (and docker) can read.
  auth=$(ps -eo args= | sed -n '/[X]org .*vt[0-9]/s|.* -auth \([^ ]*\).*|\1|p' | head -n 1)
  if [ -n "$auth" ] && [ -S /tmp/.X11-unix/X0 ]; then
    local copy="/tmp/home2-x-auth-$uid"
    if sudo -n install -o "$uid" -g "$gid" -m 600 "$auth" "$copy" 2>/dev/null \
       && DISPLAY=:0 XAUTHORITY="$copy" xset q >/dev/null 2>&1; then
      HOME2_X_DISPLAY=":0"
      HOME2_X_AUTH="$copy"
      return 0
    fi
  fi

  # 3. Whatever the shell points at, as long as that X server is alive.
  if [ -n "${DISPLAY:-}" ] && DISPLAY="$DISPLAY" xset q >/dev/null 2>&1; then
    HOME2_X_DISPLAY="$DISPLAY"
    HOME2_X_AUTH="${XAUTHORITY:-$HOME/.Xauthority}"
    return 0
  fi

  return 1
}

# Resolve the physical display, let containers running as this user draw on
# it, and keep the monitor from blanking. Exports DISPLAY and XAUTHORITY_HOST
# for docker compose. Call from an area's run.sh before `docker compose up`.
prepare_physical_display() {
  if ! resolve_physical_display; then
    echo "[display] WARNING: no usable X display found; GUIs will not open." >&2
    echo "[display] Log in on the robot's monitor, or set HOME2_DISPLAY (e.g. HOME2_DISPLAY=:10.0)." >&2
    export DISPLAY="${DISPLAY:-:0}"
    export XAUTHORITY_HOST="${XAUTHORITY:-$HOME/.Xauthority}"
    return 1
  fi

  echo "[display] Using DISPLAY=$HOME2_X_DISPLAY (Xauthority: $HOME2_X_AUTH)"
  # Containers run with our uid, so allow it by user even if the cookie in
  # the mounted Xauthority does not match (e.g. after a re-login).
  DISPLAY="$HOME2_X_DISPLAY" XAUTHORITY="$HOME2_X_AUTH" \
    xhost "+SI:localuser:$(id -un)" >/dev/null 2>&1 || true

  bash "$__HOME2_LIB_DIR/scripts/disable_screen_blanking.sh" \
    "$HOME2_X_DISPLAY" "$HOME2_X_AUTH" "$HOME2_X_SESSION_BUS" || true

  export DISPLAY="$HOME2_X_DISPLAY"
  export XAUTHORITY_HOST="$HOME2_X_AUTH"
  return 0
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

# Resolve MAP_NAME, falling back to the user's shell rc file if the env var is
# unset in the current shell (the common case when ./run.sh --update-map was
# run in the same terminal without sourcing the rc file afterwards).
resolve_map_name() {
  local default_value="${1:-lab_23_march.db}"

  if [ -n "${MAP_NAME:-}" ]; then
    echo "$MAP_NAME"
    return 0
  fi

  local rc_file="$HOME/.bashrc"
  if [ -f "$HOME/.zshrc" ]; then
    rc_file="$HOME/.zshrc"
  fi

  if [ -f "$rc_file" ]; then
    local line
    line=$(grep -E '^[[:space:]]*export[[:space:]]+MAP_NAME=' "$rc_file" | tail -n 1)
    if [ -n "$line" ]; then
      local value="${line#*=}"
      value="${value%\"}"; value="${value#\"}"
      value="${value%\'}"; value="${value#\'}"
      if [ -n "$value" ]; then
        echo "$value"
        return 0
      fi
    fi
  fi

  echo "$default_value"
}
