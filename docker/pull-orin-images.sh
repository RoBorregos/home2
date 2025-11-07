#!/bin/bash

# Script to pull Orin/Jetson Docker images from Docker Hub
# Usage: ./pull-orin-images.sh [version]
# Example: ./pull-orin-images.sh v1.0.0
# If no version specified, pulls 'latest'

set -e  # Exit on error

# Source shared utilities
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/image_utils.sh"

# Get version from argument or use 'latest'
VERSION=$(get_version "${1:-latest}")
echo -e "${GREEN}Pulling version: ${VERSION}${NC}"

# Array of images to pull
IMAGES=(
    "roborregos/home2:l4t_base"
    "roborregos/home2:hri-l4t"
    "roborregos/home2:manipulation-jetson"
    "roborregos/home2:navigation-jetson"
    "roborregos/home2:vision-jetson"
)

# Pull each image
for image in "${IMAGES[@]}"; do
    pull_image "$image" "$VERSION"
done

echo -e "\n${GREEN}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}✓ All images successfully pulled!${NC}"
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
