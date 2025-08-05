# Building the Deployment Image

## Prerequisites

- Docker and Docker Compose installed
- NVIDIA Container Toolkit (for GPU support)
- Sufficient disk space (~8-10 GB)

## Build the Image

### Using Build Script (Recommended)

```bash
cd docker/
./build.sh
```

### Manual Build

```bash
cd docker/
docker-compose build rover-deploy
```

## What Happens During Build

1. **Base Environment** - L4T image, ROS 2 Humble, PyTorch
2. **ZED SDK** - Camera support and dependencies  
3. **Source Integration** - Copies `../src` and runs `colcon build`
4. **Production Setup** - Configures entrypoint and environment

## Build Output

After successful build:
```bash
✅ Rover deployment image built successfully!

To run the deployment container:
  docker-compose up rover-deploy

To run in detached mode (background):
  docker-compose up -d rover-deploy
```

The deployment image is now ready to run continuously in the background.
