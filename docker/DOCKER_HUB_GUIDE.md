# Docker Hub Integration Guide for Orin/Jetson Images

This guide explains how to build, push, and manage Docker images for the NVIDIA Jetson Orin platform on RoBorregos Docker Hub.

## Table of Contents

- [Quick Start](#quick-start)
- [Available Images](#available-images)
- [Method 1: Automated Script (Recommended)](#method-1-automated-script-recommended)
- [Method 2: Manual Build and Push](#method-2-manual-build-and-push)
- [Method 3: GitHub Actions CI/CD](#method-3-github-actions-cicd)
- [Pulling Images](#pulling-images)
- [Version Management](#version-management)
- [Troubleshooting](#troubleshooting)

---

## Quick Start

### Prerequisites

- Docker installed on your system
- Access to RoBorregos Docker Hub account
- Jetson Orin device (for testing) or compatible build environment

### Login to Docker Hub

```bash
docker login
# Enter username and password for roborregos account
```

---

## Available Images

The following Jetson Orin/L4T images are available:

| Image Name                             | Description                    | Base Image                  |
| -------------------------------------- | ------------------------------ | --------------------------- |
| `roborregos/home2:l4t_base`            | Base L4T image with ROS Humble | dustynv/l4t-pytorch:r36.4.0 |
| `roborregos/home2:hri-l4t`             | Human-Robot Interaction module | l4t_base                    |
| `roborregos/home2:manipulation-jetson` | Manipulation/grasping module   | l4t_base                    |
| `roborregos/home2:navigation-jetson`   | Navigation/SLAM module         | l4t_base                    |
| `roborregos/home2:vision-jetson`       | Computer vision module         | l4t_base                    |

---

## Method 1: Automated Script (Recommended)

### Build and Push All Images

Use the provided script to build and push all Jetson images:

```bash
cd /path/to/home2

# Make scripts executable
chmod +x docker/push-orin-images.sh
chmod +x docker/pull-orin-images.sh

# Push with custom version
./docker/push-orin-images.sh v1.0.0

# Or push with date-based version (default)
./docker/push-orin-images.sh
```

The script will:

1. ✓ Check Docker Hub authentication
2. ✓ Build all Jetson images in correct order
3. ✓ Tag images with version and 'latest'
4. ✓ Push all tags to Docker Hub
5. ✓ Provide summary of pushed images

### Pull Images from Docker Hub

```bash
# Pull latest version
./docker/pull-orin-images.sh

# Pull specific version
./docker/pull-orin-images.sh v1.0.0
```

---

## Method 2: Manual Build and Push

### Step-by-Step Process

#### 1. Build Images Locally

```bash
cd /path/to/home2

# Build base L4T image (required first)
docker compose -f docker/l4t.yaml build

# Build HRI images
docker compose -f docker/hri/compose/docker-compose-l4t.yml build

# Build Manipulation image
docker compose -f docker/manipulation/docker-compose-jetson.yaml build

# Build Navigation image
docker compose -f docker/navigation/docker-compose-jetson.yaml build

# Build Vision image
docker compose -f docker/vision/docker-compose.yaml build
```

#### 2. Tag Images

```bash
# Define version
VERSION="v1.0.0"

# Tag base image
docker tag roborregos/home2:l4t_base roborregos/home2:l4t_base-${VERSION}
docker tag roborregos/home2:l4t_base roborregos/home2:l4t_base-latest

# Tag HRI image
docker tag roborregos/home2:hri-l4t roborregos/home2:hri-l4t-${VERSION}
docker tag roborregos/home2:hri-l4t roborregos/home2:hri-l4t-latest

# Tag Manipulation image
docker tag roborregos/home2:manipulation-jetson roborregos/home2:manipulation-jetson-${VERSION}
docker tag roborregos/home2:manipulation-jetson roborregos/home2:manipulation-jetson-latest

# Tag Navigation image
docker tag roborregos/home2:navigation-jetson roborregos/home2:navigation-jetson-${VERSION}
docker tag roborregos/home2:navigation-jetson roborregos/home2:navigation-jetson-latest

# Tag Vision image
docker tag roborregos/home2:vision-jetson roborregos/home2:vision-jetson-${VERSION}
docker tag roborregos/home2:vision-jetson roborregos/home2:vision-jetson-latest
```

#### 3. Push to Docker Hub

```bash
# Push base image
docker push roborregos/home2:l4t_base-${VERSION}
docker push roborregos/home2:l4t_base-latest

# Push HRI image
docker push roborregos/home2:hri-l4t-${VERSION}
docker push roborregos/home2:hri-l4t-latest

# Push Manipulation image
docker push roborregos/home2:manipulation-jetson-${VERSION}
docker push roborregos/home2:manipulation-jetson-latest

# Push Navigation image
docker push roborregos/home2:navigation-jetson-${VERSION}
docker push roborregos/home2:navigation-jetson-latest

# Push Vision image
docker push roborregos/home2:vision-jetson-${VERSION}
docker push roborregos/home2:vision-jetson-latest
```

---

## Method 3: GitHub Actions CI/CD

### Setup (One-time)

1. **Add Docker Hub credentials to GitHub Secrets:**

   - Go to: `https://github.com/RoBorregos/home2/settings/secrets/actions`
   - Add two secrets:
     - `DOCKER_USERNAME`: Your Docker Hub username
     - `DOCKER_PASSWORD`: Your Docker Hub password or access token

2. **The workflow file is already created at:**
   ```
   .github/workflows/docker-orin.yml
   ```

### Automatic Triggers

The workflow automatically builds and pushes images when:

1. **Code is pushed to `main` or `stable` branches** with changes in:

   - `docker/**`
   - `hri/**`, `manipulation/**`, `navigation/**`, `vision/**`
   - The workflow file itself

2. **A new release is published** on GitHub

3. **Manual trigger** via GitHub Actions UI:
   - Go to Actions tab
   - Select "Build and Push Orin Docker Images"
   - Click "Run workflow"

### Image Tagging Strategy

The GitHub Actions workflow tags images with:

- `latest` - Latest stable version
- `<branch-name>` - Branch-specific builds
- `<git-sha>` - Specific commit builds
- `<semantic-version>` - Release versions (when using releases)

### Example: Creating a Release

```bash
# Tag a release locally
git tag -a v1.0.0 -m "Release version 1.0.0"
git push origin v1.0.0

# Or create via GitHub UI
# Go to: https://github.com/RoBorregos/home2/releases/new
```

This will automatically build and push images tagged with `v1.0.0`.

---

## Pulling Images

### Pull Specific Version

```bash
# Pull specific version
docker pull roborregos/home2:l4t_base-v1.0.0
docker pull roborregos/home2:hri-l4t-v1.0.0
docker pull roborregos/home2:manipulation-jetson-v1.0.0
docker pull roborregos/home2:navigation-jetson-v1.0.0
docker pull roborregos/home2:vision-jetson-v1.0.0
```

### Pull Latest Version

```bash
# Pull latest versions
docker pull roborregos/home2:l4t_base-latest
docker pull roborregos/home2:hri-l4t-latest
docker pull roborregos/home2:manipulation-jetson-latest
docker pull roborregos/home2:navigation-jetson-latest
docker pull roborregos/home2:vision-jetson-latest
```

### Using Pulled Images

Update your `docker-compose.yaml` files to reference the specific version:

```yaml
services:
  manipulation:
    image: roborregos/home2:manipulation-jetson-v1.0.0
    # ... rest of configuration
```

Or use the pull script:

```bash
./docker/pull-orin-images.sh v1.0.0
```

---

## Version Management

### Versioning Strategy

We recommend using **Semantic Versioning** (SemVer):

- **Major version** (v1.0.0 → v2.0.0): Breaking changes
- **Minor version** (v1.0.0 → v1.1.0): New features, backward compatible
- **Patch version** (v1.0.0 → v1.0.1): Bug fixes

### Alternative: Date-based Versioning

```bash
# Format: YYYYMMDD
VERSION=$(date +%Y%m%d)
./docker/push-orin-images.sh $VERSION

# Example: 20251031
```

### View Available Tags on Docker Hub

Visit: `https://hub.docker.com/r/roborregos/home2/tags`

Or use Docker CLI:

```bash
docker search roborregos/home2 --limit 25
```

---

## Troubleshooting

### Issue: Authentication Failed

**Solution:**

```bash
# Re-login to Docker Hub
docker logout
docker login
```

### Issue: Image Build Fails

**Solution:**

```bash
# Clean Docker build cache
docker builder prune -a

# Rebuild without cache
docker compose -f docker/l4t.yaml build --no-cache
```

### Issue: Push Timeout

**Solution:**

```bash
# Increase timeout (add to ~/.docker/config.json)
{
  "experimental": "enabled",
  "max-concurrent-uploads": 1
}

# Or push images one at a time
docker push roborregos/home2:l4t_base-v1.0.0
```

### Issue: Large Image Size

**Solution:**

- Images are expected to be large (5-15GB) due to:
  - NVIDIA L4T base (4GB+)
  - ROS Humble installation (2GB+)
  - Python packages and dependencies (1-3GB+)
  - Built ROS packages (1-3GB+)

To reduce size:

```dockerfile
# Add to Dockerfile
RUN apt-get clean && rm -rf /var/lib/apt/lists/*
RUN pip cache purge
```

### Issue: Multi-architecture Builds

If you need to build for different architectures:

```bash
# Create buildx builder
docker buildx create --name mybuilder --use
docker buildx inspect --bootstrap

# Build for multiple platforms
docker buildx build \
  --platform linux/arm64,linux/amd64 \
  -t roborregos/home2:l4t_base-v1.0.0 \
  --push \
  .
```

---

## Best Practices

1. **Always test locally before pushing:**

   ```bash
   # Build and test locally
   docker compose -f docker/manipulation/docker-compose-jetson.yaml up
   # Test functionality
   # Then push to Docker Hub
   ```

2. **Use version tags for production:**

   ```yaml
   # Good: Specific version
   image: roborregos/home2:manipulation-jetson-v1.0.0

   # Avoid in production: Latest tag
   image: roborregos/home2:manipulation-jetson-latest
   ```

3. **Document changes in commit messages:**

   ```bash
   git commit -m "docker: Update manipulation Dockerfile for GPD v2.0"
   ```

4. **Create releases for stable versions:**

   - Tag releases in Git
   - Update CHANGELOG
   - Test thoroughly before release

5. **Use GitHub Actions for consistency:**
   - Automated builds ensure reproducibility
   - Same environment for all builds
   - Automatic tagging with commit SHA

---

## Additional Resources

- [Docker Hub - RoBorregos](https://hub.docker.com/u/roborregos)
- [NVIDIA L4T Containers](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-base)
- [Docker Compose Documentation](https://docs.docker.com/compose/)
- [GitHub Actions Docker Build](https://docs.github.com/en/actions/publishing-packages/publishing-docker-images)

---

## Support

For issues or questions:

- Open an issue on GitHub: `https://github.com/RoBorregos/home2/issues`
- Contact the DevOps team
- Check existing documentation in `docs/`

---

**Last Updated:** October 31, 2025
**Maintainer:** RoBorregos Team
