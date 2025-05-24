docker build --build-arg CACHEBUST=$(date +%s) -f Dockerfile-base -t base-image-rover .
