# Deployment Overview

The GORM rover software provides two deployment modes: **development** and **production**.

## Development Mode
- Source code mounted as volumes for live editing
- Manual building inside container
- Interactive development and testing

## Production Mode
- Source code copied and pre-built during image creation
- **Runs continuously in the background** with automatic restarts
- Optimized for autonomous operation

## Deployment Files

| File | Purpose |
|------|---------|
| `Dockerfile.deploy` | Production Dockerfile with pre-built workspace |
| `entrypoint.deploy.sh` | Production entrypoint with startup modes |
| `build.sh` | Build script for deployment image |
| `docker-compose.yaml` | Both development and production services |

## Quick Start

```bash
# Build production image
cd docker/
./build.sh

# Deploy (runs in background continuously)
docker-compose up -d rover-deploy
```

The production deployment will run continuously in the background unless manually stopped.