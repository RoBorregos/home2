# Quick Reference: Docker Hub for Orin Images

## 🚀 Quick Start

### Option 1: Using Scripts (Easiest)

```bash
# Build and push all images with version v1.0.0
./docker/push-orin-images.sh v1.0.0

# Pull all images version v1.0.0
./docker/pull-orin-images.sh v1.0.0

# List local images
./docker/list-orin-images.sh
```

### Option 2: Using Makefile (Recommended)

```bash
cd docker/

# See all available commands
make help

# Build and push all images
make deploy VERSION=v1.0.0

# Pull latest images
make update

# List local images
make list
```

### Option 3: GitHub Actions (Automated)

Push to `main` branch or create a release, and images are automatically built and pushed.

## 📦 Available Images

| Image        | Latest Tag                                    | Versioned Example                             |
| ------------ | --------------------------------------------- | --------------------------------------------- |
| Base L4T     | `roborregos/home2:l4t_base-latest`            | `roborregos/home2:l4t_base-v1.0.0`            |
| HRI          | `roborregos/home2:hri-l4t-latest`             | `roborregos/home2:hri-l4t-v1.0.0`             |
| Manipulation | `roborregos/home2:manipulation-jetson-latest` | `roborregos/home2:manipulation-jetson-v1.0.0` |
| Navigation   | `roborregos/home2:navigation-jetson-latest`   | `roborregos/home2:navigation-jetson-v1.0.0`   |
| Vision       | `roborregos/home2:vision-jetson-latest`       | `roborregos/home2:vision-jetson-v1.0.0`       |

## 📝 Common Commands

### Login

```bash
docker login
# or
make login
```

### Build Specific Image

```bash
# Using Makefile
make build-manipulation

# Or using docker compose
cd docker/manipulation
docker compose -f docker-compose-jetson.yaml build
```

### Push Single Image

```bash
VERSION=v1.0.0
docker tag roborregos/home2:manipulation-jetson roborregos/home2:manipulation-jetson-${VERSION}
docker push roborregos/home2:manipulation-jetson-${VERSION}
```

### Pull and Use

```bash
# Pull specific version
docker pull roborregos/home2:manipulation-jetson-v1.0.0

# Update docker-compose to use it
# In your docker-compose.yaml:
image: roborregos/home2:manipulation-jetson-v1.0.0
```

## 🔄 Workflow

1. **Development**: Make changes locally
2. **Build**: `make build-all` or `./docker/push-orin-images.sh`
3. **Test**: Run containers locally
4. **Push**: `make push-all VERSION=v1.0.0`
5. **Deploy**: Pull on Jetson device

## 📚 Full Documentation

See [DOCKER_HUB_GUIDE.md](./DOCKER_HUB_GUIDE.md) for complete documentation.

## 🆘 Quick Troubleshooting

**Can't login?**

```bash
docker logout
docker login
```

**Build fails?**

```bash
docker builder prune -a
make build-all
```

**Push timeout?**
Push images one at a time instead of using `push-all`.

## 🔗 Links

- [Docker Hub Repository](https://hub.docker.com/r/roborregos/home2)
- [GitHub Repository](https://github.com/RoBorregos/home2)
- [Full Documentation](./DOCKER_HUB_GUIDE.md)
