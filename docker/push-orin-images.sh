#!/bin/bash

# Script to build and push Orin/Jetson Docker images to Docker Hub
# Usage: ./push-orin-images.sh [version]
# Example: ./push-orin-images.sh v1.0.0

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get version from argument or use date-based version
VERSION="${1:-$(date +%Y%m%d)}"
echo -e "${GREEN}Using version: ${VERSION}${NC}"

# Check if logged into Docker Hub
if ! docker info | grep -q "Username"; then
    echo -e "${YELLOW}Not logged into Docker Hub. Logging in...${NC}"
    docker login
fi

# Function to build, tag, and push an image
build_tag_push() {
    local compose_file=$1
    local image_name=$2
    local context_dir=$3
    
    echo -e "\n${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}Processing: ${image_name}${NC}"
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    # Build the image
    echo -e "${YELLOW}Building ${image_name}...${NC}"
    if [ -n "$context_dir" ]; then
        cd "$context_dir"
    fi
    docker compose -f "$compose_file" build
    
    # Tag with version
    echo -e "${YELLOW}Tagging ${image_name}...${NC}"
    docker tag "${image_name}" "${image_name}-${VERSION}"
    docker tag "${image_name}" "${image_name}-latest"
    
    # Push both tags
    echo -e "${YELLOW}Pushing ${image_name}...${NC}"
    docker push "${image_name}-${VERSION}"
    docker push "${image_name}-latest"
    
    echo -e "${GREEN}✓ Successfully pushed ${image_name}${NC}"
}

# Store the root directory
ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
echo -e "${YELLOW}Root directory: ${ROOT_DIR}${NC}"

# Build and push base L4T image
echo -e "\n${GREEN}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}Step 1: Base L4T Image${NC}"
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
build_tag_push "${ROOT_DIR}/docker/l4t.yaml" "roborregos/home2:l4t_base" "${ROOT_DIR}"

# Build and push HRI L4T images
echo -e "\n${GREEN}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}Step 2: HRI L4T Images${NC}"
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
build_tag_push "${ROOT_DIR}/docker/hri/compose/docker-compose-l4t.yml" "roborregos/home2:hri-l4t" "${ROOT_DIR}/docker/hri/compose"

# Build and push Manipulation Jetson image
echo -e "\n${GREEN}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}Step 3: Manipulation Jetson Image${NC}"
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
build_tag_push "${ROOT_DIR}/docker/manipulation/docker-compose-jetson.yaml" "roborregos/home2:manipulation-jetson" "${ROOT_DIR}/docker/manipulation"

# Build and push Navigation Jetson image
echo -e "\n${GREEN}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}Step 4: Navigation Jetson Image${NC}"
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
build_tag_push "${ROOT_DIR}/docker/navigation/docker-compose-jetson.yaml" "roborregos/home2:navigation-jetson" "${ROOT_DIR}/docker/navigation"

# Build and push Vision Jetson image
echo -e "\n${GREEN}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}Step 5: Vision Jetson Image${NC}"
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
build_tag_push "${ROOT_DIR}/docker/vision/docker-compose.yaml" "roborregos/home2:vision-jetson" "${ROOT_DIR}/docker/vision"

# Summary
echo -e "\n${GREEN}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}✓ All images successfully built and pushed!${NC}"
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
echo -e "\nPushed images with tags:"
echo -e "  - roborregos/home2:l4t_base-${VERSION}"
echo -e "  - roborregos/home2:hri-l4t-${VERSION}"
echo -e "  - roborregos/home2:manipulation-jetson-${VERSION}"
echo -e "  - roborregos/home2:navigation-jetson-${VERSION}"
echo -e "  - roborregos/home2:vision-jetson-${VERSION}"
echo -e "\nAll images also tagged with '-latest'"
echo -e "${YELLOW}To use these images, update your docker-compose files to reference the version.${NC}"
