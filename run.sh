ARGS=("$@")  # Save all arguments in an array
AREA=${ARGS[0]}  # Get the first argument
# Check if the parameter is passed, if not, set a default value (e.g., "vision")
if [ -z "$AREA" ]; then
  echo "No service name provided or invalid. Valid args are: vision, hri, etc"
  exit 1
fi

echo "Service name is: $AREA"

# check arguments passed as --help or -h
if [ "$AREA" == "--help" ] || [ "$AREA" == "-h" ]; then
  echo "Usage: ./run.sh [service_name]"
  echo "Example: ./run.sh vision"
  exit 0
fi

area=""
rebuild=0


case $AREA in
  vision)
    echo "Running vision..."
    area="vision"
    ;;
  manipulation)
    echo "Running manipulation"
    area="manipulation"
    ;;
  navigation)
    echo "Running manipulation"
    area="navigation"
    ;;
  *)
    echo "Invalid service name provided. Valid args are: vision, hri, etc"
    exit 1
    ;;
esac

REBUILD=0
# check if one of the arguments is --rebuild
for arg in "${ARGS[@]}"; do
  if [ "$arg" == "--rebuild" ]; then
    rebuild=1
  fi
done

if [ -z "$area" ]; then
  echo "Invalid service name provided. Valid args are: vision, hri, etc"
  exit 1
fi

cd docker/$area
if [ $rebuild -eq 1 ]; then
  echo "Rebuilding image from area: $area"
  ./run.sh --rebuild
else
  echo "Running image from area: $area"
  ./run.sh
fi