#!/bin/bash

if [ "$1" == "--triton" ]; then
  # Start rover and triton-server
  echo "[INFO] Launching rover and triton-server..."
  docker compose --file docker-compose.yaml up rover triton-server --detach --build --remove-orphans
else
  # Start only rover container
  echo "[INFO] Launching rover..."
  sudo modprobe iptable_raw
  docker compose --file docker-compose.yaml up rover --detach --build --remove-orphans
fi

