#!/bin/bash

# Enhanced run script for refactored Docker structure
# Supports both development and production modes with the new multi-stage Dockerfile

print_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "MODES:"
    echo "  dev               Development mode (default) - mounts source code"
    echo "  deploy            Production mode - uses built image"
    echo ""
    echo "OPTIONS:"
    echo "  --triton          Include triton-server (works with both modes)"
    echo "  --help, -h        Show this help message"
    echo ""
    echo "EXAMPLES:"
    echo "  $0 dev                    # Development mode"
    echo "  $0 deploy                 # Production mode"
    echo "  $0 dev --triton           # Development mode + triton"
    echo "  $0 deploy --triton        # Production mode + triton"
}

# Parse arguments
TRITON=false
MODE="dev"  # Default to development mode

while [[ $# -gt 0 ]]; do
    case $1 in
        dev)
            MODE="dev"
            shift
            ;;
        deploy)
            MODE="deploy"
            shift
            ;;
        --triton)
            TRITON=true
            shift
            ;;
        --help|-h)
            print_usage
            exit 0
            ;;
        *)
            echo "[ERROR] Unknown option: $1"
            print_usage
            exit 1
            ;;
    esac
done

# Determine compose file and services
COMPOSE_FILE="docker-compose.yaml"
SERVICES=""

if [ "$MODE" = "deploy" ]; then
    echo "[INFO] Using production deployment mode..."
    SERVICES="rover-deploy"
    
    if [ "$TRITON" = true ]; then
        SERVICES="$SERVICES triton-server"
        echo "[INFO] Including triton-server..."
    fi
    
    echo "[INFO] Launching production rover..."
    sudo modprobe iptable_raw 2>/dev/null || true
    docker compose --file $COMPOSE_FILE up $SERVICES --detach --build --remove-orphans
else
    echo "[INFO] Using development mode..."
    SERVICES="rover"
    
    if [ "$TRITON" = true ]; then
        SERVICES="$SERVICES triton-server"
        echo "[INFO] Including triton-server..."
    fi
    
    echo "[INFO] Launching development rover..."
    sudo modprobe iptable_raw 2>/dev/null || true
    docker compose --file $COMPOSE_FILE up $SERVICES --detach --build --remove-orphans
fi

echo "[INFO] Launch complete!"
echo "[INFO] Use 'docker compose -f $COMPOSE_FILE logs -f' to view logs"
echo "[INFO] Use 'docker compose -f $COMPOSE_FILE down' to stop services"
echo "[INFO] Use 'docker exec -it rover bash' to enter the container"
