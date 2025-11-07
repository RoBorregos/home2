#!/bin/bash

# Shared utilities for pushing and pulling Docker images
# This script provides modular functions for image management

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get version from argument or use date-based version
get_version() {
    local version="${1}"
    if [ -z "$version" ]; then
        # No version provided, use date
        echo "$(date +%Y%m%d)"
    else
        # Use provided version
        echo "$version"
    fi
}

# Check if logged into Docker Hub
check_docker_login() {
    if ! docker info | grep -q "Username"; then
        echo -e "${YELLOW}Not logged into Docker Hub. Logging in...${NC}"
        docker login
    fi
}

# Function to push a single image
push_image() {
    local image_name=$1
    local version=$2
    
    echo -e "\n${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}Pushing: ${image_name}${NC}"
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    # Check if image exists locally
    if ! docker images --format "{{.Repository}}:{{.Tag}}" | grep -q "^${image_name}$"; then
        echo -e "${RED}✗ Image ${image_name} not found locally. Please build it first.${NC}"
        return 1
    fi
    
    # Tag with version
    echo -e "${YELLOW}Tagging ${image_name}...${NC}"
    docker tag "${image_name}" "${image_name}-${version}"
    docker tag "${image_name}" "${image_name}-latest"
    
    # Push both tags
    echo -e "${YELLOW}Pushing ${image_name}...${NC}"
    docker push "${image_name}-${version}"
    docker push "${image_name}-latest"
    
    echo -e "${GREEN}✓ Successfully pushed ${image_name}${NC}"
    return 0
}

# Function to pull a single image
pull_image() {
    local image_name=$1
    local version=$2
    
    echo -e "\n${YELLOW}Pulling ${image_name}-${version}...${NC}"
    
    if docker pull "${image_name}-${version}"; then
        # Tag as the base name (without version suffix) for local use
        docker tag "${image_name}-${version}" "${image_name}"
        echo -e "${GREEN}✓ Successfully pulled ${image_name}${NC}"
        return 0
    else
        echo -e "${RED}✗ Failed to pull ${image_name}-${version}${NC}"
        return 1
    fi
}

# Function to build and push an image from docker-compose
build_and_push_image() {
    local compose_file=$1
    local image_name=$2
    local context_dir=$3
    local version=$4
    
    echo -e "\n${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}Building and Pushing: ${image_name}${NC}"
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    # Change to context directory if specified
    if [ -n "$context_dir" ]; then
        pushd "$context_dir" > /dev/null || return 1
    fi
    
    # Build the image
    echo -e "${YELLOW}Building ${image_name}...${NC}"
    if ! docker compose -f "$compose_file" build; then
        echo -e "${RED}✗ Failed to build ${image_name}${NC}"
        [ -n "$context_dir" ] && popd > /dev/null
        return 1
    fi
    
    # Return to original directory
    [ -n "$context_dir" ] && popd > /dev/null
    
    # Push the image
    push_image "$image_name" "$version"
    return $?
}

# Function to get image name for an area and environment
get_image_name() {
    local area=$1
    local env_type=$2
    
    case $area in
        manipulation)
            echo "roborregos/home2:manipulation-${env_type}"
            ;;
        navigation)
            echo "roborregos/home2:navigation-${env_type}"
            ;;
        vision)
            echo "roborregos/home2:vision-${env_type}"
            ;;
        hri)
            echo "roborregos/home2:hri-${env_type}"
            ;;
        *)
            echo ""
            return 1
            ;;
    esac
}

# Function to get compose file for an area and environment
get_compose_file() {
    local area=$1
    local env_type=$2
    local root_dir=$3
    
    case $area in
        manipulation)
            echo "${root_dir}/docker/${area}/docker-compose-${env_type}.yaml"
            ;;
        navigation|vision)
            # Navigation and vision use a single docker-compose.yaml file with env vars
            echo "${root_dir}/docker/${area}/docker-compose.yaml"
            ;;
        hri)
            if [ "$env_type" = "l4t" ]; then
                echo "${root_dir}/docker/hri/compose/docker-compose-l4t.yml"
            else
                echo "${root_dir}/docker/hri/compose/docker-compose-${env_type}.yaml"
            fi
            ;;
        *)
            echo ""
            return 1
            ;;
    esac
}

# Main push function for an area
push_area_image() {
    local area=$1
    local env_type=$2
    local version=$3
    local root_dir=$4
    
    check_docker_login
    
    local image_name
    image_name=$(get_image_name "$area" "$env_type")
    if [ -z "$image_name" ]; then
        echo -e "${RED}✗ Unknown area: ${area}${NC}"
        return 1
    fi
    
    local compose_file
    compose_file=$(get_compose_file "$area" "$env_type" "$root_dir")
    if [ -z "$compose_file" ]; then
        echo -e "${RED}✗ Could not determine compose file for ${area}${NC}"
        return 1
    fi
    
    if [ ! -f "$compose_file" ]; then
        echo -e "${RED}✗ Compose file not found: ${compose_file}${NC}"
        return 1
    fi
    
    local context_dir
    context_dir=$(dirname "$compose_file")
    
    # For navigation and vision, we need to set up environment variables
    if [ "$area" = "navigation" ] || [ "$area" = "vision" ]; then
        pushd "$context_dir" > /dev/null || return 1
        
        # Determine dockerfile and base image based on env_type
        case $env_type in
            cpu)
                export DOCKERFILE="docker/${area}/Dockerfile.cpu"
                export BASE_IMAGE="roborregos/home2:cpu_base"
                export IMAGE_NAME="roborregos/home2:${area}-cpu"
                ;;
            cuda|gpu)
                export DOCKERFILE="docker/${area}/Dockerfile.cuda"
                export BASE_IMAGE="roborregos/home2:cuda_base"
                export IMAGE_NAME="roborregos/home2:${area}-cuda"
                ;;
            jetson)
                export DOCKERFILE="docker/${area}/Dockerfile.jetson"
                export BASE_IMAGE="roborregos/home2:l4t_base"
                export IMAGE_NAME="roborregos/home2:${area}-jetson"
                ;;
        esac
        
        popd > /dev/null
    fi
    
    build_and_push_image "$compose_file" "$image_name" "$context_dir" "$version"
}

# Main pull function for an area
pull_area_image() {
    local area=$1
    local env_type=$2
    local version=$3
    
    local image_name
    image_name=$(get_image_name "$area" "$env_type")
    if [ -z "$image_name" ]; then
        echo -e "${RED}✗ Unknown area: ${area}${NC}"
        return 1
    fi
    
    pull_image "$image_name" "$version"
}

# Export functions for use in other scripts
export -f get_version
export -f check_docker_login
export -f push_image
export -f pull_image
export -f build_and_push_image
export -f get_image_name
export -f get_compose_file
export -f push_area_image
export -f pull_area_image
