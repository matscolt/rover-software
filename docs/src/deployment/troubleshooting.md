# Quick Reference

## Basic Commands

```bash
# Build deployment image
cd docker/ && ./build.sh

# Start production deployment (runs in background)
docker-compose up -d rover-deploy

# View logs
docker-compose logs -f rover-deploy

# Stop deployment
docker-compose down rover-deploy

# Access running container
docker-compose exec rover-deploy bash
```

## Common Issues

**Container won't start:** Check `docker-compose logs rover-deploy`

**Need to rebuild:** Run `./build.sh` then restart with `docker-compose down rover-deploy && docker-compose up -d rover-deploy`

**Check if running:** `docker-compose ps rover-deploy`
