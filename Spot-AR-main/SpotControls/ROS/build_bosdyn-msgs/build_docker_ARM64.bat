:: Set up environment for linux
docker buildx create --use --name multiarchbuilder
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

:: Build for ARM64 on Windows
docker build -t build-bosdyn-msgs-arm64 -f Dockerfile.ARM64 --platform linux/arm64  .
