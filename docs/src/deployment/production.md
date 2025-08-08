# Running in Production

## Start Production Deployment

**Basic deployment (runs continuously in background):**
```bash
cd docker/
docker-compose up -d rover-deploy
```

**The production container will:**
- Start immediately and run continuously in the background
- Automatically restart if it crashes (`restart: unless-stopped`) 
- Only stop when manually commanded

## Startup Modes

**Default (autostart teleop):**
```bash
docker-compose up -d rover-deploy
```

**Custom startup:**
```bash
# Standard bringup (no teleop)
docker-compose run --rm rover-deploy bringup

# Interactive shell
docker-compose run --rm rover-deploy bash
```

## Managing the Deployment

**View logs:**
```bash
docker-compose logs -f rover-deploy
```

**Check status:**
```bash
docker-compose ps rover-deploy
```

**Stop the deployment:**
```bash
docker-compose down rover-deploy
```

**Restart:**
```bash
docker-compose restart rover-deploy
```

**Access running container:**
```bash
docker-compose exec rover-deploy bash
```

## Updating Deployment

To deploy new code:
```bash
./build.sh
docker-compose down rover-deploy
docker-compose up -d rover-deploy
```

## Background Operation

**Important:** The production deployment is designed to run continuously in the background. It will:
- Start automatically when the system boots (if configured)
- Run unattended without user interaction
- Automatically restart if the rover software crashes
- Continue running until explicitly stopped

This makes it suitable for autonomous operations, competitions, and field deployments where the rover needs to operate reliably without manual intervention.
