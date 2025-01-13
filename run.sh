INPUT=$1  # If no parameter is passed, $1 will be empty

# Check if the parameter is passed, if not, set a default value (e.g., "vision")
if [ -z "$INPUT" ]; then
  echo "No service name provided or invalid. Valid args are: vision, hri, etc"
  exit 1
fi

echo "Service name is: $INPUT"

# check arguments passed as --help or -h
if [ "$INPUT" == "--help" ] || [ "$INPUT" == "-h" ]; then
  echo "Usage: ./run.sh [service_name]"
  echo "Example: ./run.sh vision"
  exit 0
fi

area=""
rebuild=0

case $INPUT in
  vision)
    echo "Running vision..."
    area="vision"
    ;;
  manipulation)
    echo "Running manipulation"
    area="manipulation"
    ;;
  rebuild)
    echo "Rebuilding all services..."
    rebuild=1
    ;;
  *)
    echo "Invalid service name provided. Valid args are: vision, hri, etc"
    exit 1
    ;;
esac

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