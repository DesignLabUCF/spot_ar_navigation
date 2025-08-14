:: Credit
:: https://dev.bostondynamics.com/docs/payload/docker_containers#create-docker-images

:: Set up environment for linux
docker buildx create --use --name multiarchbuilder
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

:: Build for ARM64 on Windows
docker build -t spot-ros-bdai-arm64 -f Dockerfile.ARM64 --platform linux/arm64  .

:: Save to TGZ file
docker save spot-ros-bdai-arm64 > spot-ros-bdai-arm64.tgz