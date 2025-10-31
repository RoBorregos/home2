#!/bin/bash

# Script to list available Docker images and their versions on Docker Hub
# Usage: ./list-orin-images.sh

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}RoBorregos Home2 - Orin/Jetson Docker Images${NC}"
echo -e "${GREEN}═══════════════════════════════════════════${NC}\n"

# Array of images to check
IMAGES=(
    "roborregos/home2:l4t_base"
    "roborregos/home2:hri-l4t"
    "roborregos/home2:hri-stt-l4t"
    "roborregos/home2:manipulation-jetson"
    "roborregos/home2:navigation-jetson"
    "roborregos/home2:vision-jetson"
)

echo -e "${BLUE}Local Images:${NC}\n"

# Check each image locally
for image_base in "${IMAGES[@]}"; do
    image_name="${image_base##*:}"
    echo -e "${YELLOW}${image_name}:${NC}"
    
    # Get local tags
    local_tags=$(docker images --format "{{.Repository}}:{{.Tag}}" | grep "^${image_base%:*}:" | grep -E "${image_name}|latest" | sed 's/.*:/  - /' || echo "  (not found locally)")
    
    if [ "$local_tags" != "  (not found locally)" ]; then
        echo "$local_tags"
    else
        echo "  (not found locally)"
    fi
    echo ""
done

echo -e "\n${BLUE}To view all available tags on Docker Hub:${NC}"
echo -e "Visit: ${YELLOW}https://hub.docker.com/r/roborregos/home2/tags${NC}\n"

echo -e "${BLUE}To pull a specific version:${NC}"
echo -e "${YELLOW}./docker/pull-orin-images.sh <version>${NC}"
echo -e "Example: ${YELLOW}./docker/pull-orin-images.sh v1.0.0${NC}\n"

echo -e "${BLUE}To pull latest version:${NC}"
echo -e "${YELLOW}./docker/pull-orin-images.sh${NC}\n"
