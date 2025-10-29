AREA=$1

# check arguments passed as --help or -h
if [ "$AREA" == "--help" ] || [ "$AREA" == "-h" ] || [ -z "$AREA" ]; then
  echo "Usage: ./run.sh [area] [--task] [--flags]"
  echo "Example: ./run.sh hri --receptionist --open-display"
  exit 0
fi

case $AREA in
  vision|manipulation|navigation|integration|hri|frida_interfaces)
    ;;
  *)
    echo "Invalid service name provided. Valid args are: vision, manipulation, navigation, integration, hri, frida_interfaces"
    exit 1
    ;;
esac

# Function to check if an image exists
check_image_exists() {
    local image_name=$1
    if ! docker images --format "{{.Repository}}:{{.Tag}}" | grep -q "^${image_name}$"; then
        echo "Image $image_name does not exist. Building it..."
        return 1  # Image doesn't exist
    else
        echo "Image $image_name already exists. Skipping build."
        return 0  # Image exists
    fi
}

# Check type of environment (cpu, cuda, or l4t), default cpu
ENV_TYPE=cpu

if [ -f /etc/nv_tegra_release ]; then
  ENV_TYPE=l4t
elif command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
  ENV_TYPE=cuda
fi
echo "Detected environment: $ENV_TYPE"

# Check and build base image if it doesn't exist
check_image_exists "roborregos/home2:${ENV_TYPE}_base"
if [ $? -eq 1 ]; then
  docker compose -f docker/${ENV_TYPE}.yaml build
fi

# Run the selected area
if [ "$AREA" = "frida_interfaces" ]; then
  echo "Running frida_interfaces_cache to build frida_interfaces"
  docker compose -f docker/frida_interfaces_cache/docker-compose-${ENV_TYPE}.yaml run --rm frida_interfaces_cache
else
  # If frida_interfaces_cache hasn't been built yet, build it first
  if [ ! -d "docker/frida_interfaces_cache/build" ]; then
    echo "Cache directory missing. Running frida_interfaces_cache to build frida_interfaces"
    docker compose -f docker/frida_interfaces_cache/docker-compose-${ENV_TYPE}.yaml run --rm frida_interfaces_cache
  fi

  echo "Running image from area: $AREA"
  cd "docker/$AREA" || { echo "Error: failed to cd into docker/$AREA" >&2; exit 1; }
  ./run.sh "${@:2}"
fi