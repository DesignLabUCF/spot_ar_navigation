:: Set up environment for linux
::docker buildx create --use --name multiarchbuilder
::docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

:: Compose extension
docker-compose up