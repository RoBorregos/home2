#!/bin/bash

# Script to pull Orin/Jetson Docker images from Docker Hub
# Usage: ./pull-orin-images.sh [version]
# Example: ./pull-orin-images.sh v1.0.0
# If no version specified, pulls 'latest'

set -e  # Exit on error

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get version from argument or use 'latest'
VERSION="${1:-latest}"
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
    echo -e "\n${YELLOW}Pulling ${image}-${VERSION}...${NC}"
    docker pull "${image}-${VERSION}"
    
    # Tag as the base name (without version suffix) for local use
    docker tag "${image}-${VERSION}" "${image}"
    echo -e "${GREEN}✓ Successfully pulled ${image}${NC}"
done

echo -e "\n${GREEN}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}✓ All images successfully pulled!${NC}"
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
