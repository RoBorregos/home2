#!/bin/bash

# Script to build and push Orin/Jetson Docker images to Docker Hub
# Usage: ./push-orin-images.sh [version]
# Example: ./push-orin-images.sh v1.0.0

set -e  # Exit on error

# Source shared utilities
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/image_utils.sh"

# Get version from argument or use date-based version
VERSION=$(get_version "$1")
echo -e "${GREEN}Using version: ${VERSION}${NC}"

# Check if logged into Docker Hub
check_docker_login

# Store the root directory
ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
echo -e "${YELLOW}Root directory: ${ROOT_DIR}${NC}"

# Build and push base L4T image
echo -e "\n${GREEN}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}Step 1: Base L4T Image${NC}"
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
build_and_push_image "${ROOT_DIR}/docker/l4t.yaml" "roborregos/home2:l4t_base" "${ROOT_DIR}" "${VERSION}"

# Build and push HRI L4T images
echo -e "\n${GREEN}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}Step 2: HRI L4T Images${NC}"
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
build_and_push_image "${ROOT_DIR}/docker/hri/compose/docker-compose-l4t.yml" "roborregos/home2:hri-l4t" "${ROOT_DIR}/docker/hri/compose" "${VERSION}"

# Build and push Manipulation Jetson image
echo -e "\n${GREEN}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}Step 3: Manipulation Jetson Image${NC}"
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
build_and_push_image "${ROOT_DIR}/docker/manipulation/docker-compose-jetson.yaml" "roborregos/home2:manipulation-jetson" "${ROOT_DIR}/docker/manipulation" "${VERSION}"

# Build and push Navigation Jetson image
echo -e "\n${GREEN}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}Step 4: Navigation Jetson Image${NC}"
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
build_and_push_image "${ROOT_DIR}/docker/navigation/docker-compose-jetson.yaml" "roborregos/home2:navigation-jetson" "${ROOT_DIR}/docker/navigation" "${VERSION}"

# Build and push Vision Jetson image
echo -e "\n${GREEN}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}Step 5: Vision Jetson Image${NC}"
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
build_and_push_image "${ROOT_DIR}/docker/vision/docker-compose.yaml" "roborregos/home2:vision-jetson" "${ROOT_DIR}/docker/vision" "${VERSION}"

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
