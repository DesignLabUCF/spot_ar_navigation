:: Credit
:: https://dev.bostondynamics.com/docs/payload/docker_containers#create-docker-images

:: Set up environment for linux
docker buildx create --use --name multiarchbuilder
::mkdir prebuilt
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

:: Build for ARM64 on Windows
:: docker build -t ping-image-dockerization-arm64 --platform linux/arm64 .
docker build -t ping-image-dockerization-arm64 --platform linux/arm64 --build-arg BOSDYN_CLIENT_USERNAME=%BOSDYN_CLIENT_USERNAME% --build-arg BOSDYN_CLIENT_PASSWORD=%BOSDYN_CLIENT_PASSWORD% .

:: Save to TGZ file
docker save ping-image-dockerization-arm64 > ping-image-dockerization-arm64.tgz