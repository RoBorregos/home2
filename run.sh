AREA=$1

# check arguments passed as --help or -h
if [ "$AREA" == "--help" ] || [ "$AREA" == "-h" ] || [ -z "$AREA" ]; then
  echo "Usage: ./run.sh [area] [--task] [--flags]"
  echo "Example: ./run.sh hri --receptionist --open-display"
  exit 0
fi

case $AREA in
  vision|manipulation|navigation|integration|hri|zed)
    ;;
  *)
    echo "Invalid service name provided. Valid args are: vision, manipulation, navigation, integration, hri, zed"
    exit 1
    ;;
esac


if [ "$AREA" == "zed" ]; then
  echo "Running zed2"
  cd docker/integration
  docker compose -f zed.yml up $detached
  exit 0
fi

cd docker/$AREA
echo "Running image from area: $area"
./run.sh "${@:2}"