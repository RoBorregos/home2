# Makefile for building and managing Orin/Jetson Docker images
# Usage: make <target>

.PHONY: help build-base build-hri build-manipulation build-navigation build-vision
.PHONY: build-all push-all pull-all clean list login

# Default version - override with: make push-all VERSION=v1.0.0
VERSION ?= $(shell date +%Y%m%d)

# Docker Hub organization
ORG = roborregos
REPO = home2

# Image names
BASE_IMAGE = $(ORG)/$(REPO):l4t_base
HRI_IMAGE = $(ORG)/$(REPO):hri-l4t
MANIP_IMAGE = $(ORG)/$(REPO):manipulation-jetson
NAV_IMAGE = $(ORG)/$(REPO):navigation-jetson
VISION_IMAGE = $(ORG)/$(REPO):vision-jetson

# Colors for output
GREEN = \033[0;32m
YELLOW = \033[1;33m
NC = \033[0m # No Color

help:
	@echo "$(GREEN)═══════════════════════════════════════════$(NC)"
	@echo "$(GREEN) RoBorregos Home2 - Docker Management$(NC)"
	@echo "$(GREEN)═══════════════════════════════════════════$(NC)"
	@echo ""
	@echo "$(YELLOW)Build targets:$(NC)"
	@echo "  make build-base          - Build L4T base image"
	@echo "  make build-hri           - Build HRI L4T image"
	@echo "  make build-manipulation  - Build Manipulation Jetson image"
	@echo "  make build-navigation    - Build Navigation Jetson image"
	@echo "  make build-vision        - Build Vision Jetson image"
	@echo "  make build-all           - Build all images"
	@echo ""
	@echo "$(YELLOW)Push/Pull targets:$(NC)"
	@echo "  make push-all            - Tag and push all images"
	@echo "  make pull-all            - Pull all images from Docker Hub"
	@echo ""
	@echo "$(YELLOW)Utility targets:$(NC)"
	@echo "  make login               - Login to Docker Hub"
	@echo "  make list                - List local images"
	@echo "  make clean               - Remove all local images"
	@echo ""
	@echo "$(YELLOW)Examples:$(NC)"
	@echo "  make build-all                    # Build all images"
	@echo "  make push-all VERSION=v1.0.0      # Push with custom version"
	@echo "  make pull-all VERSION=v1.0.0      # Pull specific version"
	@echo ""
	@echo "$(YELLOW)Current version:$(NC) $(VERSION)"

login:
	@echo "$(GREEN)Logging into Docker Hub...$(NC)"
	@docker login

build-base:
	@echo "$(GREEN)Building L4T base image...$(NC)"
	@docker compose -f docker/l4t.yaml build

build-hri: build-base
	@echo "$(GREEN)Building HRI L4T image...$(NC)"
	@cd docker/hri/compose && docker compose -f docker-compose-l4t.yml build

build-manipulation: build-base
	@echo "$(GREEN)Building Manipulation Jetson image...$(NC)"
	@cd docker/manipulation && docker compose -f docker-compose-jetson.yaml build

build-navigation: build-base
	@echo "$(GREEN)Building Navigation Jetson image...$(NC)"
	@cd docker/navigation && docker compose -f docker-compose-jetson.yaml build

build-vision: build-base
	@echo "$(GREEN)Building Vision Jetson image...$(NC)"
	@cd docker/vision && docker compose -f docker-compose.yaml build

build-all: build-base build-hri build-manipulation build-navigation build-vision
	@echo "$(GREEN)✓ All images built successfully!$(NC)"

tag-all: build-all
	@echo "$(GREEN)Tagging images with version: $(VERSION)$(NC)"
	@docker tag $(BASE_IMAGE) $(BASE_IMAGE)-$(VERSION)
	@docker tag $(BASE_IMAGE) $(BASE_IMAGE)-latest
	@docker tag $(HRI_IMAGE) $(HRI_IMAGE)-$(VERSION)
	@docker tag $(HRI_IMAGE) $(HRI_IMAGE)-latest
	@docker tag $(MANIP_IMAGE) $(MANIP_IMAGE)-$(VERSION)
	@docker tag $(MANIP_IMAGE) $(MANIP_IMAGE)-latest
	@docker tag $(NAV_IMAGE) $(NAV_IMAGE)-$(VERSION)
	@docker tag $(NAV_IMAGE) $(NAV_IMAGE)-latest
	@docker tag $(VISION_IMAGE) $(VISION_IMAGE)-$(VERSION)
	@docker tag $(VISION_IMAGE) $(VISION_IMAGE)-latest
	@echo "$(GREEN)✓ All images tagged!$(NC)"

push-all: tag-all
	@echo "$(GREEN)Pushing images to Docker Hub...$(NC)"
	@docker push $(BASE_IMAGE)-$(VERSION)
	@docker push $(BASE_IMAGE)-latest
	@docker push $(HRI_IMAGE)-$(VERSION)
	@docker push $(HRI_IMAGE)-latest
	@docker push $(MANIP_IMAGE)-$(VERSION)
	@docker push $(MANIP_IMAGE)-latest
	@docker push $(NAV_IMAGE)-$(VERSION)
	@docker push $(NAV_IMAGE)-latest
	@docker push $(VISION_IMAGE)-$(VERSION)
	@docker push $(VISION_IMAGE)-latest
	@echo "$(GREEN)✓ All images pushed successfully!$(NC)"
	@echo ""
	@echo "$(YELLOW)Pushed images:$(NC)"
	@echo "  - $(BASE_IMAGE)-$(VERSION)"
	@echo "  - $(HRI_IMAGE)-$(VERSION)"
	@echo "  - $(MANIP_IMAGE)-$(VERSION)"
	@echo "  - $(NAV_IMAGE)-$(VERSION)"
	@echo "  - $(VISION_IMAGE)-$(VERSION)"

pull-all:
	@echo "$(GREEN)Pulling images from Docker Hub (version: $(VERSION))...$(NC)"
	@docker pull $(BASE_IMAGE)-$(VERSION) && docker tag $(BASE_IMAGE)-$(VERSION) $(BASE_IMAGE)
	@docker pull $(HRI_IMAGE)-$(VERSION) && docker tag $(HRI_IMAGE)-$(VERSION) $(HRI_IMAGE)
	@docker pull $(MANIP_IMAGE)-$(VERSION) && docker tag $(MANIP_IMAGE)-$(VERSION) $(MANIP_IMAGE)
	@docker pull $(NAV_IMAGE)-$(VERSION) && docker tag $(NAV_IMAGE)-$(VERSION) $(NAV_IMAGE)
	@docker pull $(VISION_IMAGE)-$(VERSION) && docker tag $(VISION_IMAGE)-$(VERSION) $(VISION_IMAGE)
	@echo "$(GREEN)✓ All images pulled successfully!$(NC)"

list:
	@echo "$(GREEN)Local Orin/Jetson images:$(NC)"
	@docker images | grep -E "roborregos/home2.*(l4t|jetson)" || echo "No images found"

clean:
	@echo "$(YELLOW)Removing all local Orin/Jetson images...$(NC)"
	@docker images | grep -E "roborregos/home2.*(l4t|jetson)" | awk '{print $$3}' | xargs -r docker rmi -f || true
	@echo "$(GREEN)✓ Clean complete!$(NC)"

# Quick deploy - build, tag, and push with one command
deploy: build-all push-all
	@echo "$(GREEN)✓ Deployment complete!$(NC)"

# Pull latest and retag as base names
update:
	@$(MAKE) pull-all VERSION=latest
	@echo "$(GREEN)✓ Update complete! Local images now use latest from Docker Hub.$(NC)"
