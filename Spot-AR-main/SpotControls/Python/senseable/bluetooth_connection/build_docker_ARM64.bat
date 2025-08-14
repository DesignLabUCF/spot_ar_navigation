:: Credit
:: https://dev.bostondynamics.com/docs/payload/docker_containers#create-docker-images

:: Set up environment for linux
docker buildx create --use --name multiarchbuilder
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

:: Build for ARM64 on Windows
docker build -t bluetooth-arm64 --platform linux/arm64 .

:: Save to TGZ file
docker save bluetooth-arm64 > bluetooth-arm64.tgz