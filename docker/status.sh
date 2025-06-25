

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'
BLUE_BG_WHITE='\033[44;1;37m'

RED_BG='\033[41m'  # Red background
echo -e "${RED_BG}${WHITE} CRITICAL ERROR ${NC}"

echo -e "${BLUE_BG_WHITE}=== Vision Status ${TASK} ===${NC}"

# Get currently running ROS 2 nodes
RUNNING_NODES=$(ros2 node list 2>/dev/null)

if [ -z "$RUNNING_NODES" ]; then
  echo -e "${RED}No ROS 2 nodes are running!${NC}"
  exit 1
fi

echo "=== ROS 2 Node Status Check ==="
echo "Expected nodes: ${EXPECTED_NODES[*]}"
echo "-----------------------------"

# Check each expected node
MISSING_NODES=()
for node in "${EXPECTED_NODES[@]}"; do
  if grep -q "$node" <<< "$RUNNING_NODES"; then
    echo "✅ $node: RUNNING"
  else
    echo "❌ $node: NOT RUNNING (expected)"
    MISSING_NODES+=("$node")
  fi
done

# Summary
echo -e "\n=== SUMMARY ==="
echo "Total expected nodes: ${#EXPECTED_NODES[@]}"
echo "Running: $((${#EXPECTED_NODES[@]} - ${#MISSING_NODES[@]}))"
echo "Missing: ${#MISSING_NODES[@]}"

if [ ${#MISSING_NODES[@]} -gt 0 ]; then
  echo -e "\n[WARNING] These nodes should be running but are missing:"
  printf '  - %s\n' "${MISSING_NODES[@]}"
fi