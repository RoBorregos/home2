INPUT=$1  # If no parameter is passed, $1 will be empty

# Check if the parameter is passed, if not, set a default value (e.g., "vision")
if [ -z "$INPUT" ]; then
  echo "No service name provided or invalid. Valid args are: vision, hri, etc"
  exit 1
fi

echo "Service name is: $INPUT"

case $INPUT in
  vision)
    echo "Running vision..."
    cd docker/vision
    ./run.sh
    ;;

esac